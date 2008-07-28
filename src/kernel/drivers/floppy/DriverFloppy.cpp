/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * This was implemented using to the Intel 82077A data sheet, but I have not used
 * any extended commands; the driver should thus work with even the oldest of FDC
 * controllers.
 *
 * This is 1.44MB only; other types are detected and reported.
 *
 * Other sources used for the floppy driver:
 *
 *  [1] Undocumented PC, 2nd edition
 *  [2] IHWB, 4th edition
 *
 * \todo Migrate floppy driver to the C++ driver framework
 * \bug No error handling (especially the Channel interface doesn't return FALSE... ever')
 * \bug Formatting doesn't work yet
 * \bug Media changed handling must be improved (uhm, or implemented...)
 */

#include <kernel/drivers/floppy/DriverFloppy.h>
#include <kernel/drivers/system/DMA.h>
#include <kernel/init/irq.h>
#include <kernel/io/Ports.h>
#include <kernel/threads/ThreadScheduler.h>
#include <kernel/drivers/timer/Timer.h>
#include <kernel/vmm/malloc.h>
#include <kernel/util/Math.h>
#include <kernel/Errors.h>

namespace
{
    #define CMOS_REG_DISKDRIVE_TYPE     0x10
}

/** \brief Floppy params */
static struct FloppyParameters _fdcParams;
static DriverFloppy* _fdcFloppies[FLOPPY_MAX_DRIVES];
static int _floppyCount = 0;
static bool _irqReceived = FALSE;

/** \brief Called when a floppy interrupt is raised */
static boolean onFloppyInterrupt(ContextState *ctx);

/** \brief Official floppy drive types */
static char *_floppyTypes[] =
{
    "Not detected",
    "360 KB 5.25 inch",
    "1.2 MB 5.25 inch",
    "720 KB 3.5 inch",
    "1.44 MB 3.5 inch",
    "2.88 MB 3.5 inch",
};

/**
 * C'tor
 */
FloppyChannel::FloppyChannel(DriverFloppy* dev)
{
    this->dev = dev;
    this->position = 0;
}

/**
 * Implements Channel#setPosition
 */
error_t FloppyChannel::setPosition(fpos_t position)
{
    this->position = position;
    return E_OK;
}

/** Clone the channel */
AbstractChannel* FloppyChannel::clone()
{
    FloppyChannel* copy = new FloppyChannel(dev);
    *copy = *this;
    copy->setPosition(0);

    return copy;
}


/**
 * Implements Channel#read
 */
ssize_t FloppyChannel::read(uint8* buff, size_t length, bool block/*=true*/)
{
    int res = E_OK;
    uint8* buffer = (uint8*)buff;

    // Compute LBA's
    int firstSector = position / dev->actualBytesPerSector;
    int lastSector = (position + length) / dev->actualBytesPerSector;

    //out << "[FDC-READ] POS="<<position<< ", FS=" << firstSector << ", LS=" << lastSector << ", LEN=" << length << eol;

    // TODO: just debugging
    kassert_stmt(firstSector <= lastSector && lastSector < (2880),
                 out << "firstSector = " << firstSector << ", lastSector=" << lastSector << eol);

    if(firstSector > lastSector || lastSector >= 2880)
        return -Thread::setLastError(E_RANGE);

    // This is stupid... the floppyTranser thingy should be able to
    // work directly on the input buffer, rather than copying each
    // sector (first and last partially) again...
    char* tmpBuf = (char*) kmalloc(dev->actualBytesPerSector);

    // Read sectors. Now, most drivers do this by first copying the first partial sector,
    // then the "middle" sectors and finally the partial last sector. Here we simply
    // apply a simple loop and some basic arithmetic...
    for(int i=firstSector; i <= lastSector; i++)
    {
        int offset = position % dev->actualBytesPerSector;

        floppyReadSector(dev, tmpBuf, i);

        int segment = Math::min((int)length, (dev->actualBytesPerSector - offset));
        position += segment;

        memcpy(buffer, tmpBuf + offset, segment);

        buffer += segment;
        length -= segment;
    }

    kfree(tmpBuf);

    return res;
}

/**
 * Implements Channel#write
 */
ssize_t FloppyChannel::write(const uint8* buff, size_t length, bool block/*=true*/)
{
    int res = E_OK;

    uint8* buffer = (uint8*)buff;

    // Compute LBA's
    int firstSector = position / dev->actualBytesPerSector;
    int lastSector = (position + length) / dev->actualBytesPerSector;

    // out << "[FDC-WRITE] POS="<<position<< ", FS=" << firstSector << ", LS=" << lastSector << ", LEN=" << length << eol;

    // TODO: Just debugging
    kassert(firstSector <= lastSector && lastSector < (2880));

    if(firstSector > lastSector || lastSector >= 2880)
    {
        return -Thread::setLastError(E_RANGE);
    }

    unsigned char* tmpBuf = (unsigned char*)kmalloc(dev->actualBytesPerSector);

    // Read, change and write sectors
    for(int i=firstSector; i <= lastSector; i++)
    {
        int offset = position % dev->actualBytesPerSector;

        floppyReadSector(dev, tmpBuf, i);

        int segment = Math::min((int)length, (dev->actualBytesPerSector - offset));
        position += segment;

        memcpy(tmpBuf + offset, buffer, segment);

        floppyWriteSector(dev, tmpBuf, i);

        //tmpBuf[60] = '\0';
        //out << "  [FDC-WRITE] Offset " << offset << ", Segment " << segment << eol;
        //out << "  [FDC-WRITE] Org buf: " << (char*)buffer << eol;
        //out << "  [FDC-WRITE] New Buf: " << (char*)tmpBuf << eol;

        buffer += segment;
        length -= segment;
    }

    kfree(tmpBuf);

    return res;

}


/**
 * Initialize driver
 *
 * \pre Paging must be up
 */
void DriverFloppy::initialize()
{
    // Get drive types from CMOS
    out8(CMOS_PORT_OUT, CMOS_REG_DISKDRIVE_TYPE);
    unsigned char driveTypes = in8(CMOS_PORT_IN);

    // Drive one type in high nibble, drive two in low nibble. Note that this
    // restricts the number of drives to two.
    int types[2] = {driveTypes >> 4,
                    driveTypes & 0x0F };

    for(int i=0; i < FLOPPY_MAX_DRIVES; i++)
    {
        _fdcFloppies[i] = new DriverFloppy(&_fdcParams);
        installIRQHandler(IRQ_SYSTEM_FDC, _fdcFloppies[i]);

        if(types[i] != 0)
        {
            // Only supports 1.44 for now
            out << "Floppy " << (i+1) << ": " << _floppyTypes[types[i]] << eol;
            kassert(types[i] == 4);

            // Initialize
            _fdcFloppies[_floppyCount]->driveNumber = _floppyCount;

            // Reset floppy driver
            floppyReset(_fdcFloppies[_floppyCount]);

            //
            // TODO: register device in device manager. NO: the base class Driver c'tor should do that
            //

            _floppyCount++;
        }
    }

    // Print controller version for the first floppy drive
    floppySendCommand(_fdcFloppies[0], FDC_CMD_VERSION);

    if(floppyGetByte(_fdcFloppies[0]) == 0x80)
    {
        out << "NEC765 FDC Detected";
    }
    else
    {
        out << "Enhanced FDC Detected";
    }

    out.println();
}

/**
 * Called on boot-time to grab the floppy parameter block
 */
void copyFloppyParams()
{
    memcpy( (void*)&_fdcParams,
            (const void*) FLOPPY_PARAMETER_ADDRESS,
            sizeof(struct FloppyParameters));
}

struct DriverFloppy* getFloppy(int index)
{
    if(index < _floppyCount)
    {
        return _fdcFloppies[index];
    }
    else
    {
        return null;
    }
}

int DriverFloppy::onInterrupt(ContextState* ctx)
{
    _irqReceived = TRUE;
    return IRQRES_NORMAL;
}

bool floppyWaitIRQ()
{
    kassert(irqEnabled() == TRUE);

    // Wait up to 10 seconds, in 10 ms intervals
    for(int i=0; i < 1000; i++)
    {
        if((volatile int*) _irqReceived)
            break;

        Thread::sleep(10);
    };

    // OK if irq received
    return _irqReceived;
}

/**
 * Spins until data is ready, using a heuristically determined retry value
 */
bool floppyWaitDataReady(DriverFloppy* dev)
{
    int retries=0;
    for(; retries < FDC_RETRIES_MAX; retries++)
    {
        int msr = in8(dev->portBase+FDC_REG_MSR);

        if(IS_FLAG_SET(msr, FDC_MSR_BIT_DIO) &&
           IS_FLAG_SET(msr, FDC_MSR_BIT_MRQ) &&
           IS_FLAG_SET(msr, FDC_MSR_BIT_BUSY))
        {
            break;
        }
    }

    // Success only if retries wasn't exhausted
    return (retries < FDC_RETRIES_MAX);
}

/**
 * \param resPtr Only ST0 and cylinder is updated after this command
 */
void floppySenseInterrupt(DriverFloppy* dev)
{
    floppySendCommand(dev, FDC_CMD_SENSE_INTERRUPT);
    floppyWaitDataReady(dev);

    int val = floppyGetByte(dev);

    if(val == -1)
    {
        out << "FDC [SENSEI]: Operation timed out" << eol;
    }
    else
    {
        dev->cmdResult.ST0 = (uint8_t) val;
        floppyWaitDataReady(dev);
        dev->cmdResult.cylinder = floppyGetByte(dev);
    }

    kassert(irqEnabled() == TRUE);
}

void floppyReset(DriverFloppy* dev)
{
    // Stop and disable irq's. We output directly, since floppyStop... has a delay.
    out8(((DriverFloppy*)dev)->portBase+FDC_REG_DOR, 0x00);

    floppyStartMotor(dev);

    if(dev->motorOn == FALSE)
    {
        out << "[FDC " << dev->driveNumber << "] Couldn't start floppy motor" << eol;
        dev->lastError = FDC_ERR_NO_RESET;
        return;
    }

    // Set data rate
    out8(dev->portBase+FDC_REG_DRS, 0);

    // Reset and reenable interrupts
    out8(dev->portBase+FDC_REG_DOR, (FDC_DOR_VAL_RESET | FDC_DOR_VAL_DMAGATE) );

    // Initialize FDC timers
    floppySendCommand(dev, FDC_CMD_SPECIFY);
    floppyPutByte(dev, dev->fdcParams->SRT_HUT);
    floppyPutByte(dev, dev->fdcParams->HUT_NDMA);

    floppySeek(dev, 1);
    floppyRecalibrate(dev);
    floppyStopMotor(dev);
}

/**
 * FIXME: This is untested code
 */
bool floppyMediaChanged(DriverFloppy* dev)
{
    uint8_t dir = in8(dev->portBase+FDC_REG_DIR);
    dev->mediaChanged = (IS_FLAG_SET(dir, FDC_DIR_FLAG_MEDIACHANGE));

    return dev->mediaChanged;
}

bool floppySendCommand(DriverFloppy* dev, uint8_t command)
{
    _irqReceived = FALSE;
    floppyPutByte(dev, command);

    return TRUE;
}

bool floppyGetResult(DriverFloppy* dev)
{
    bool res = TRUE;

    uint8_t *statusEntry = (unsigned char *) &dev->cmdResult;

    for(int i=0; i < 7; i++)
    {
        int byte = floppyGetByte(dev);

        if(byte == -1)
        {
            out << "FDC [GET RESULT]: Operation timed out" << eol;
            res = FALSE;
            break;
        }

        statusEntry[i] = (uint8_t) res;
    }

    // Check media change after each command
    if(res)
    {
        // TODO: safe to do this without SENSEI first?
        floppyMediaChanged(dev);
    }

    return res;
}

bool floppyRecalibrate(DriverFloppy* dev)
{
    bool res = TRUE;

    floppyStartMotor(dev);

    // RECALIBRATE ::= CMD DRIVE
    floppySendCommand(dev, FDC_CMD_RECALIBRATE);
    floppyPutByte(dev, dev->driveNumber);

    // Wait for interrupt
    if(floppyWaitIRQ())
    {
        dev->recalibrated = TRUE;

        floppySenseInterrupt(dev);
    }
    else
        out << "FDC: No IRQ received" << eol;

    if(dev->cmdResult.cylinder != 0)
    {
        out << "FDC: Recalibrate failed" << eol;
    }

    floppyStopMotor(dev);

    return TRUE;
}

// UNTESTED
void floppySenseDriveStatus(DriverFloppy* dev)
{
    floppySendCommand(dev, FDC_CMD_SENSE_DRIVE_STATUS);
    floppyPutByte(dev, dev->driveNumber);

    int st3 = in8(dev->portBase+FDC_REG_DOR);

    // TODO: now what? return st3?
}

bool floppyIsMotorOn(DriverFloppy* dev)
{
    int dor = in8(dev->portBase+FDC_REG_DOR);
    int motorFlag = FDC_DOR_VAL_MOTOR_A << dev->driveNumber;

    return (dor & motorFlag);
}

void floppyStartMotor(DriverFloppy* dev)
{
    flag_t flags;
    disableLocalIRQ(&flags);

    // Flag ASAP in case a stop is scheduled
    dev->motorOn = TRUE;

    // Trust only the FDC about running status (the internal flag is just for
    // scheduling motor off)
    if(!floppyIsMotorOn(dev))
    {
        int motorFlag = FDC_DOR_VAL_MOTOR_A << dev->driveNumber;

        out8(dev->portBase+FDC_REG_DOR,
            FDC_DOR_VAL_RESET | FDC_DOR_VAL_DMAGATE | motorFlag | dev->driveNumber);

        // Motor Start Time is specified in 1/8'th secs
        for(int i=0; i < 3; i++)
        {
            Thread::sleep(dev->fdcParams->MST * (1000/8));

            if(floppyIsMotorOn(dev))
            {
                break;
            }
        }
    }

    // Couldn't start...
    if(!floppyIsMotorOn(dev))
    {
        dev->motorOn = FALSE;
    }

    restoreLocalIRQ(flags);
}

void onFloppyStop(void* dev)
{
    // floppyStopMotor() will set this to FALSE. If someone starts the motor in the
    // meantime, it means we shouldn't stop it after all.
    if(((DriverFloppy*)dev)->motorOn == FALSE)
    {
        // Send reset to stop all motors and disable interrupts
        out8(((DriverFloppy*)dev)->portBase+FDC_REG_DOR, 0x00);
    }
}

void floppyStopMotor(DriverFloppy* dev)
{
    // Don't issue timers unless we have to
    if(dev->motorOn)
    {
        // Schedule stop
        dev->motorOn = FALSE;

        // Stop in 3 seconds. Note that if dev->motorOn is changed to TRUE
        // in the mean time, the floppy will NOT be stopped.
        Timer::addTimer(FDC_MOTOR_STOP_DELAY_MS, onFloppyStop, (void*) dev);
    }
}

bool floppySeek(DriverFloppy* dev, int lba)
{
    int sector, cylinder, head;
    floppyGetCHS(dev, lba, &sector, &cylinder, &head);

    return floppySeekEx(dev, cylinder, head);
}

bool floppySeekEx(DriverFloppy* dev, int cylinder, int head)
{
    floppyStartMotor(dev);

    floppySendCommand(dev, FDC_CMD_SEEK);
    floppyPutByte(dev, (head << 2) | dev->driveNumber);
    floppyPutByte(dev, cylinder);

    floppyWaitIRQ();
    floppySenseInterrupt(dev);

    // TODO: Check PCN (present cyl number) in ST0
    bool isOK = ( IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_SEEK_END) &&
                 !IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC1) &&
                 !IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC0));

    // Adhere to head settling time (see 8.2 in 82077AA)
    Thread::sleep(dev->fdcParams->HST);
    floppyStopMotor(dev);

    // TODO: set errno
    kassert(isOK);

    return isOK;
}

void floppyReadSector(DriverFloppy* dev, void* buffer, int lba)
{
    floppyTransfer(dev, buffer, lba, FM_READ);
}


void floppyWriteSector(DriverFloppy* dev, void* buffer, int lba)
{
    floppyTransfer(dev, buffer, lba, FM_WRITE);
}

void floppyTransfer(DriverFloppy* dev, void* buffer, int lba, enum FloppyMode mode)
{
    int sector, cylinder, head;
    floppyGetCHS(dev, lba, &sector, &cylinder, &head);

    // DMA
    void* dmaBuffer = aquireDMABuffer();
    memset(dmaBuffer, 0, dev->actualBytesPerSector);

    if(mode == FM_WRITE)
    {
        memcpy(dmaBuffer, buffer, dev->actualBytesPerSector);
    }

    dmaSetup(dmaBuffer, dev->actualBytesPerSector, (mode == FM_READ) ? DMA_READ : DMA_WRITE);

    // Set data rate
    out8(dev->portBase+FDC_REG_CCR, 0);

    // Make sure motor is on and start sending the command and parameters
    floppyStartMotor(dev);
    floppySendCommand(dev, (mode == FM_READ) ? FDC_CMD_READ_SECTORS : FDC_CMD_WRITE_SECTORS);

    // Write xxx HEAD DS1 DS0
    floppyPutByte(dev, (head << 2) | dev->driveNumber);

    // C,H,R,N
    floppyPutByte(dev, cylinder);
    floppyPutByte(dev, head);
    floppyPutByte(dev, sector);
    floppyPutByte(dev, dev->fdcParams->bytesPerSectorCode);
    floppyPutByte(dev, dev->fdcParams->sectorsPerTrack);

    // GAP 3
    floppyPutByte(dev, dev->fdcParams->gapLength);
    floppyPutByte(dev, dev->fdcParams->dataLength);

    floppyWaitIRQ();
    floppyGetResult(dev);

    floppyStopMotor(dev);

    if(!IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC1) &&
       !IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC0))
    {
        // Copy from DMA buffer to arg buffer
        // TODO: User-mode copy?
        if(mode == FM_READ)
        {
            memcpy(buffer, dmaBuffer, dev->actualBytesPerSector);
        }
    }
    else
    {
        // TODO: set last error and return FALSE
        out << "FLOPPY TRANSFER FAILED!" << eol;

    }
    // TODO: else { recalibrate and retry }

    // Done with buffer
    dmaDisableChannel(2);
    releaseDMABuffer(dmaBuffer);
}

/** \bug Doesn't work yet... at all */
void floppyFormat(DriverFloppy* dev)
{
    int sector=0, cylinder=0, head=0;
    bool isError = FALSE;

    // DMA
    uint8_t* dmaBuffer = (uint8_t*) aquireDMABuffer();

    floppyStartMotor(dev);

    // Set data rate
    out8(dev->portBase+FDC_REG_CCR, 0);

    floppyRecalibrate(dev);

    // Because the data sheet suggests so:
    Thread::sleep(500);

    ::kprintf("Formatting %d sectors\n", dev->sectorCount);

    for(cylinder=0; !isError && cylinder < dev->tracks; cylinder++)
    {
        ::kprintf("CYLINDER %d\n", cylinder);

        floppySeekEx(dev, cylinder, 0);

        for(head=0; head < FLOPPY_NUMHEADS; head++)
        {
            ::kprintf("%d,", head);

            for(int k=0; k < dev->fdcParams->sectorsPerTrack; k++)
            {
                /*if(floppyMediaChanged(dev))
                {
                    // TODO: errno
                    ::kprintf("MEDIA CHANGED - FORMATTING STOPPED\n");
                    floppyReset(dev);
                    goto error;
            }*/

                // Put C,H,R,N values in DMA buff (this becomes the Sector ID). Note that
                // CHR values are 1-based this time...
                dmaBuffer[k*4]   = cylinder;
                dmaBuffer[k*4+1] = head;
                dmaBuffer[k*4+2] = sector+1;
                dmaBuffer[k*4+3] = dev->fdcParams->bytesPerSectorCode;

                // C,H,R,N
                //floppyPutByte(dev, cylinder);
                //floppyPutByte(dev, head);
                //floppyPutByte(dev, sector++);
                //floppyPutByte(dev, dev->fdcParams->bytesPerSectorCode);
            }

            dmaSetup(dmaBuffer, dev->fdcParams->sectorsPerTrack*4 /* CHRN values for each track */, DMA_READ);

            // Write COMMAND and HEAD DS1 DS0
            floppySendCommand(dev, FDC_CMD_FORMAT);

            if(!floppyPutByte(dev, (head << 2) | dev->driveNumber) ||
               !floppyPutByte(dev, dev->fdcParams->bytesPerSectorCode)  ||
               !floppyPutByte(dev, dev->fdcParams->sectorsPerTrack) ||             // Sectors to format (value suggested by the 82077A data sheet)
               !floppyPutByte(dev, dev->fdcParams->gapLength) ||
               !floppyPutByte(dev, 'F'))
            {
                ::kprintf("FDC: FORMATTING ABORTED\n");
                isError = TRUE;
                break;
            }

            floppyWaitIRQ();
            floppyGetResult(dev);

            dmaDisableChannel(2);

            // Check status
            if(IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC1) &&
               IS_FLAG_SET(dev->cmdResult.ST0, FDC_ST0_IC0))
            {
                // TODO: set errno
                ::kprintf("FDC: FORMATTING FAILED\n");
                isError = TRUE;
                break;
            }

        }// foreach track

        /*if(floppyMediaChanged(dev))
        {
                // TODO: errno
            ::kprintf("MEDIA CHANGED - FORMATTING STOPPED\n");
            floppyReset(dev);
            break;
    }*/

    }// foreach head

    floppyStopMotor(dev);

    // Done with buffer
    releaseDMABuffer(dmaBuffer);
}

void floppyGetCHS(DriverFloppy* dev, int lba, int* sector, int* cylinder, int* head)
{
    // Yes, there is basically only way to do this...
    // Hardcoded two heads...
    *sector = (lba % dev->fdcParams->sectorsPerTrack) + 1;
    *cylinder = (lba / dev->fdcParams->sectorsPerTrack) / FLOPPY_NUMHEADS;
    *head = (lba / dev->fdcParams->sectorsPerTrack) % FLOPPY_NUMHEADS;
}

int floppyGetByte(DriverFloppy* dev)
{
    // Default is timeout
    int res = -1;

    for(int retries=0; retries < FDC_RETRIES_MAX; retries++)
    {
        // Read Main Status Register and make sure DIO and MRQ is set before
        // reading from the data register
        int msr = in8(dev->portBase+FDC_REG_MSR);

        if(IS_FLAG_SET(msr, FDC_MSR_BIT_DIO) &&
           IS_FLAG_SET(msr, FDC_MSR_BIT_MRQ))
        {
            res = in8(dev->portBase+FDC_REG_DATA);
            break;
        }
    }

    return res;
}

bool floppyPutByte(DriverFloppy* dev, uint8_t val)
{
    int res = FALSE;

    for(int retries=0; retries < FDC_RETRIES_MAX; retries++)
    {
        // Read Main Status Register and make sure MRQ is set and DIO is cleared
        // before writing to the data register
        int msr = in8(dev->portBase+FDC_REG_MSR);

        if(!IS_FLAG_SET(msr, FDC_MSR_BIT_DIO) &&
            IS_FLAG_SET(msr, FDC_MSR_BIT_MRQ))
        {
            out8(dev->portBase+FDC_REG_DATA, val);
            res = TRUE;
            break;
        }
    }

    return res;
}

#if 0
char* floppyToString(Object* obj)
{
    DriverFloppy* dev = (DriverFloppy*) obj;

    // DUH: make a growable unicode character buffer
    char* res = (char*) kmalloc(200);
    strcpy(res, "FLOPPY INFORMATION:\n\n");
    strcat(res, "...\n");

    /*
    kprintf(" steprate_headunload=%d\n", (int)dev->fdcParams->SRT_HUT);
    kprintf(" headload_ndma=%d\n", (int)dev->fdcParams->HUT_NDMA);
    kprintf(" motor_delay_off=%d\n" , (int) dev->fdcParams->motorDelayOff);
    kprintf(" bytes_per_sector=%d\n", (int) dev->actualBytesPerSector);
    kprintf(" sectors_per_track=%d\n",(int) dev->fdcParams->sectorsPerTrack);
    kprintf(" gap_length=%d\n",(int) dev->fdcParams->gapLength);
    kprintf(" data_length=%d\n",(int) dev->fdcParams->dataLength);
    kprintf(" format_gap_length=%d\n",(int) dev->fdcParams->formatGapLength);
    kprintf(" filler=%d\n",(int) dev->fdcParams->filler);

    kprintf(" head_settle_time=%d\n",(int) dev->fdcParams->HST);
    kprintf(" motor_start_time=%d\n",(int) dev->fdcParams->MST);
    */

    return res;
}
#endif
