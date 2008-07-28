/*
 * Apox Operating System
 * Copyright (c) 2005, 2006, 2007, cryptocode
 *
 * Implements the default ATA device driver. Each instance of this class drives
 * a single device via its channel (which is dropped in upon creation by the ATA
 * Bus driver code.)
 *
 * Primary reference is [S 1]
 */

#include <assert.h>
#include <kernel/drivers/ata/ATADriver.h>
#include <kernel/Errors.h>
#include <kernel/drivers/fs/generic/Partitions.h>
#include <kernel/drivers/DeviceManager.h>
#include <arch/Delay.h>

#include <kernel/util/Hexdump.h>

#define APOX_ATA_VERBOSE
#define LOG_PREFIX  "[ATA] "

/**
 * C'tor
 *
 * @param portbase ATA channel I/O port base (e.g. 0x1F0 or 0x170)
 * @param drivenum ATA channel drive selector (0 or 1)
 */
ATADriver::ATADriver(ATAChannel* channel, uint8 drivenum)
{
    kassert(sizeof(ATADeviceInfo) == 512);

    this->channel = channel;
    this->drivenum = drivenum;
    this->lastError = 0;
    this->txState = ATA_TXS_READY;

    // This should be dynamic
    this->sectorSize = 512;

    // Possibly overriden in setup()
    this->maxSectors = 256;

    this->capacity = 0;
    this->amode = ATA_MODE_INVALID;
    this->pmode = ATA_PMODE_INVALID;

    // Invalidate cache
    for(int i=0; i < ATA_CACHE_SIZE; i++)
        cache[i].lba = uint64(-1);

    cacheHits = cacheMisses = 0;
}

/** D'tor */
ATADriver::~ATADriver()
{
}

/**
 * Probes for an ATA(PI) device on the given controller/drivenum slot and, upon
 * success, return a driver instance for the device.
 *
 * @returns Device driver instance or NULL if no device was found
 */
ATADriver* ATADriver::initialize(ATAChannel* channel, uint8 drivenum)
{
    ATADriver* driver = new ATADriver(channel, drivenum);
    installIRQHandler(channel->irqline, driver);

    log << "[ATA] Probing channel " << H(channel->iobase) << ", drive "
        << drivenum << "..." << eol;

    if(!driver->identify())
    {
        log << "   Not detected" << eol;

        // clean up
        uninstallIRQHandler(channel->irqline, driver);
        delete driver;
        driver = null;
    }
    else
    {
        // Now that we have detected a device, set it up using the (P)IDENTIFY
        // info
        driver->setup();

        // Register device
        Device* ataDev = new Device();

        String name("ATA#");
        name << H(channel->iobase) << ":" << drivenum;
        if(driver->isPacket)
            name << " (ATAPI)";
        else
            name << " (ATA)";

        ataDev->setName(name);
        ataDev->setDriver(driver);

        DeviceManager::addDevice(ataDev);

        // Register in global namespace. TODO: could make Device's override
        // getChannel, and register it directly.
        ATANode* ataNode = new ATANode(name, driver);
        Node* hdNode = Kernel::getNamespace()->find(L"/dev/hd");
        assert(hdNode != null);
        hdNode->addChild(ataNode);
    }

    return driver;
}

/**
 * Check presence and issue IDENTIFY or PIDENTIFY to collect info on the devices
 *
 * @returns True if the device was present, else false
 */
bool ATADriver::identify()
{
    bool res = false;

    startTransaction();

    // Try to IDENTIFY even when DRDY is unasserted, as long as the device is
    // not busy Note that waitReady doesn't work here: it'll time out on Qemu
    // (maybe some real machines too? dunno)
    if(waitNotBusy(ATA_TIMEOUT_1000_MS*1000) == E_OK)
    {
        for(int trial=0; !res && trial < 2; trial++)
        {
            // TODO: mask out IRQ to prevent getting shared IRQ between setting
            // the flag and actually sending the command?
            txState = ATA_TXS_AWAITING_IRQ;
            out8(channel->iobase+ATA_REG_DEVSEL, (drivenum == 0 ? 0xA0 : 0xB0));
            writeCommand(trial == 0 ? ATA_CMD_PIDENTIFY_DEVICE : ATA_CMD_IDENTIFY_DEVICE);

            // Wait for interrupt (at least 35 seconds... we SERIOUSLY need a
            // pre-detection scheme here so we don't hang for a very long time
            // on some machines.)
            if(waitIRQ(500*1000) == E_OK)
            {
                if(wait(ATA_STATUS_DRQ) == E_OK)
                {
                    res = true;

                    if(trial == 0)
                    {
                        isPacket = true;
                        log << "   ATAPI detected" << eol;
                    }
                    else
                    {
                        isPacket = false;
                        log << "   ATA detected" << eol;
                    }

                    memset(&ataInfo, 0, sizeof(ATADeviceInfo));

                    uint16* info = (uint16*) &ataInfo;

                    // Read the IDENTIFY result "sector", 'sectorSize' bytes
                    // word by word, into the ATADeviceInfo struct
                    for(size_t word = 0; word != sectorSize/sizeof(uint16); word++)
                    {
                        *info = in16(channel->iobase);
                        info++;
                    }

                    // Nasty, since there isn't room for a null terminator in
                    // the ID buffer
                    fixATAString(ataInfo.model, 40, true);
                    log << "   Model: " << (char*)ataInfo.model << eol;

                    // Display some more stuff
                    if(Bits::isSet(ataInfo.config, 7))
                    {
                        log << "   Removable media" << eol;
                    }
                }
            }
            else
            {
                log << "   IRQ timeout" << eol;
            }
        }
    }
    else
    {
        log << "   Skipping (P)IDENTIFY: Drive not ready" << eol;
    }

    endTransaction();

    return res;
}

/**
 * Setup device info we need later on
 */
void ATADriver::setup()
{
    // First thing we need to do is figure out the level of LBA support, and
    // drop back to CHS addressing if necessary
    if(Bits::isSet(ataInfo.capabilities0, 9))
    {
        if(Bits::isSet(ataInfo.cmdSet2, 10))
        {
            amode = ATA_MODE_LBA48;
            maxSectors = 65*1024;
            log << "   Mode: LBA48";
        }
        else
        {
            amode = ATA_MODE_LBA28;
            log << "   Mode: LBA28";
        }
    }
    else
    {
        amode = ATA_MODE_CHS;
        log << "   Mode: CHS";
    }

    // We need to query ATAPI devices for capacity explicitely
    if(!isPacket)
    {
        // Figure out capacity
        // TODO: read native max address (ext) - maybe the actual capacity is reduced
        if(amode == ATA_MODE_CHS)
        {
            log << " (C:" << ataInfo.cylinders
                << ", H:" << ataInfo.heads
                << ", S:" << ataInfo.sectors << ")";

            capacity = ataInfo.cylinders * ataInfo.heads * ataInfo.sectors * sectorSize;
        }
        else if(amode == ATA_MODE_LBA28)
        {
            capacity = MAKE_UINT64( MAKE_UINT32(ataInfo.lba48max[0],
                                                ataInfo.lba48max[1]),
                                    MAKE_UINT32(ataInfo.lba48max[2],
                                                ataInfo.lba48max[3])) * sectorSize;

            uint32 sectors = MAKE_UINT32(ataInfo.lba28max[0], ataInfo.lba28max[1]);

            if(sectors < 1 || sectors > 0xFFFFFFF)
                log << eol << "   [ATA ERROR] Invalid LBA28 sector range reported by IDENTIFY DEVICE" << eol;

            // The reported sector count is one more than the user addressable count
            capacity = (uint64)(sectors-1)  * sectorSize;
        }
        else if(amode == ATA_MODE_LBA48)
        {
            uint64 sectors = MAKE_UINT64( MAKE_UINT32(ataInfo.lba48max[0],
                                                      ataInfo.lba48max[1]),
                                          MAKE_UINT32(ataInfo.lba48max[2],
                                                      ataInfo.lba48max[3]));

            if(sectors < 1 || sectors > 0xFFFFFFFFFFFFULL)
                log << "[ATA ERROR] Invalid LBA48 sector range reported by IDENTIFY DEVICE" << eol;

            // The reported sector count is one more than the user addressable count
            capacity = (sectors-1) * sectorSize;
        }

        log << "   Capacity: " << capacity << " bytes ("
            << (capacity/1024/1024) <<" MiB)" << eol;
    }
    else
        // Just debugging... how to get capacity from ATAPI devices?
        capacity = -1;

    // Figure out supported PIO modes
    log << "   PIO modes supported: ";

    // Can we even check the pioModes field an expect sane values?
    if(ataInfo.validFields & 1)
    {
        if(ataInfo.pioModes & 1)
            log << " [3]";
        if(ataInfo.pioModes & 2)
            log << " [4]";
    }
    else
    {
        // not allowed to ask
        log << "  [x]";
    }

    // Sectors per interrupt for READ/WRITE MULTIPLE
    log << "   Sectors per interrupt for multiread and -write: "
        << (ataInfo.sectorsPerIRQ & 0xFF) << eol;

    // Need to issue SET MULTIPLE MODE first?
    if((ataInfo.multiSectorSettings & 0x1FF) == 0x100)
    {
        log << "   Setting multisector mode..." << eol;

        // TODO: actually do it
        NO_IMPL();
    }

    // Figure out level of DMA support
    if(Bits::isSet(ataInfo.capabilities0, 8))
    {
        log << "   DMA supported" << eol;

        // TODO: more info here
    }

    // ATA versions
    if(ataInfo.majorVersion == 0x0000 || ataInfo.majorVersion == 0xFFFF)
    {
        log << "   ATA/ATAPI version support level not reported" << eol;
    }
    else
    {
        log << "   ATA/ATAPI versions supported: " << eos;

        for(int i=3; i < 15; i++)
        {
            if(Bits::isSet(ataInfo.majorVersion, i))
            {
                if(log.size())
                    log << ", ";

                log << i;
            }
        }
    }

    // Flush any remaining logs
    if(log.size())
        log << eol;
}


/**
 * Resets a packet device. This method has no effect on non-packet devices.
 *
 * TODO: untestet... when to use?
 */
void ATADriver::resetDevice()
{
    if(isPacket)
    {
        out8(channel->iobase+ATA_REG_DEVSEL, (drivenum == 0 ? 0xA0 : 0xB0));
        out8(channel->iobase+ATA_REG_COMMAND, ATA_CMD_DEVICE_RESET);

        Delay::ns(400);
    }
}

/**
 * Prints hex + ascii output; 0 shown as blanks
 */
void ATADriver::debugPrintSectors(const uint8* buf, int sectors)
{
    // Output sector by sector rather than char-by-char which would be immensly
    // slow using stdout
    char sector[sectorSize+1];
    sector[sectorSize] = 0;

    for(int i=0; i < sectors; i++)
    {
        // process byte-by-byte
        for(int j=0; j < this->sectorSize; j++)
        {
            sector[j] = buf[i*sectorSize + j];
            if(sector[j] == 0)
                sector[j] = ' ';
        }

        log << sector << eos;
    }

    log << "\nDone" << eol;
}

/**
 * Self test
 */
void ATADriver::test()
{
// DO NOT TEST NOW
return;

    const int buffsize = 256*1024+1;

    uint8* buf = new uint8[buffsize];
    memset(buf,0,buffsize);

    // Read the MBR
    readSectors(0, 1, buf);
    // Is it indeed an MBR?
    if(buf[510] == 0x55 && buf[511] == 0xAA)
        log << "MBR DETECTED\n";
    else
        log << "MBR NOT DETECTED:" << H((buf[510]<<8) & buf[511]) << eol ;
    debugPrintSectors(buf, 1);

    // Gr0k Partition table
    uint8* pt = &buf[446];
    for(int i=0; i < 4; i++)
    {
        // it's little endian
        uint64 size = (uint32)pt[12] | ((uint32)pt[13] << 8) |
                     ((uint32)pt[14] << 16) | ((uint32)pt[15]<<24);

        log << "Partion table "<<i<<" has type " << H(pt[4]) << ", size is "
            << ((size*sectorSize)/1000/1000) << " MB" << eol;
        // next table
        pt+=16;
    }

    //
    // Read tons of sectors, starting with #4
    //
    log << "\n - - - -" << eol;
    int sects = 34;
    memset(buf,0,buffsize);
    readSectors(5, sects, buf);
    //debugPrintSectors(buf, sects);
    log << "Done reading " << sects << " sectors" << eol;

    log << "First 2 sectors: " << eol;
    debugPrintSectors(buf, 2);


    // - - - TEST WRITING ON THE UNUSED TEST DISKS ONLY!!! - - -
    if(amode == ATA_MODE_CHS || strncmp((const char*)ataInfo.model, "QEMU", 4) == 0)
    {
        log << "TESTING WRITE SECTOR" << eol;
        memset(buf,0,buffsize);
        strcpy((char*)buf, "Just testing");
        writeSectors(5, 20, buf);

        // try reading it back in
        memset(buf,0,buffsize);
        readSectors(5, 20, buf);

        log << "READ BACK (data for first sector read): " << eol;
        debugPrintSectors(buf, 1);
    }
}

/* TODO: readSectors / writeSectors only differ by xfer type -> extract method */

/**
 * Read sectors using the current access method
 */
error_t ATADriver::readSectors(uint64 lba, int sectors, uint8* buffer)
{
    error_t res = E_OK;
    int lbaOffset=0;

    // Number of max-sector transfers (this is 0 when sectors < maxSectors)
    int xfersFull = (sectors / this->maxSectors);

    // Number of residue sectors (this is 0 iff sectors is a multiple of maxSectors)
    int xferExtra = (sectors % this->maxSectors);

    // Transfer full sectors
    for(int i=0; res == E_OK && i < xfersFull; i++)
    {
        res = xferPIO(ATA_XFER_READ, lba+lbaOffset, maxSectors,
                      buffer+(lbaOffset * sectorSize));
        lbaOffset += maxSectors;
    }

    // Remaining sectors
    if(res == E_OK)
    {
        res = xferPIO(ATA_XFER_READ, lba+lbaOffset, xferExtra,
                      buffer+(lbaOffset * sectorSize));
    }

    return res;
}

/**
 * Write sectors using the current access method
 */
error_t ATADriver::writeSectors(uint64 lba, int sectors, uint8* buffer)
{
    error_t res = E_OK;
    int lbaOffset=0;

    // Number of max-sector transfers (this is 0 when sectors < maxSectors)
    int xfersFull = (sectors / this->maxSectors);

    // Number of residue sectors (this is 0 iff sectors is a multiple of maxSectors)
    int xferExtra = (sectors % this->maxSectors);

    // Transfer full sectors
    for(int i=0; res == E_OK && i < xfersFull; i++)
    {
        res = xferPIO(ATA_XFER_WRITE, lba+lbaOffset, maxSectors,
                      buffer+(lbaOffset * sectorSize));
        lbaOffset += maxSectors;
    }

    // Remaining sectors
    if(res == E_OK)
    {
        res = xferPIO(ATA_XFER_WRITE, lba+lbaOffset, xferExtra,
                      buffer+(lbaOffset * sectorSize));
    }

    return res;
}

/** Hide details of writing commands so we don't forget to delay 400 ns, etc */
void ATADriver::writeCommand(int cmd)
{
    //Delay::ns(400);
    out8(channel->iobase+ATA_REG_COMMAND, cmd);

    // We have to wait 400 ns before touching the regs
    Delay::ns(400);
}

/**
 * Read 1-256/65K sectors into buffer.
 *
 * TODO: Use the ATACache. Compute ratio with
 *
 *      long hitratio = cacheHits*100 / (cacheMisses + cacheHits);
 *
 * TODO: NO! Implement cache in block layer. That layer will only call xferPIO
 *       for the sectors that aren't cached.
 *
 *                       size_t slot = (lba+s) % CACHE_SIZE;
                        if(CACHE_SIZE && cache[slot].lba == lba+s)
                        {
                            cacheHits++;
                            long hitratio = cacheHits*100 / (cacheMisses + cacheHits);
                            memcpy(buffer + (s*sectorSize), cache[slot].data, sectorSize);
                        }
                        else
                        {
                            cacheMisses++;

                            ... read data from block device ...

                            // Cache it
                            cache[slot].lba = lba+s;
                            memcpy((void*)cache[slot].data, buffer + (s*sectorSize), sectorSize);
                        }
 *
 * @lba Logical offset; converted to CHS if required
 * @rw Whether to read or write (ATA_XFER_...)
 * @sectors Number of sectors to read
 * @buffer Where to read or write - *you* make sure it is big enough
 */
error_t ATADriver::xferPIO(int rw, uint64 lba, int sectors, uint8* buffer)
{
    startTransaction();

    // Upper layer is buggy if it called xferPIO on a packet device
    kassert(!isPacket);

    error_t res = E_OK;

    if(waitReady() == E_OK)
    {
        uint8 cmd=0;

        // Use READ MULTIPLE if it can transfer > 1 sectors per interrupt
        // This equates to the DRQ Data Block definition
        // TODO: Does read-multiple even work with CHS addressing (which implies pre-ata-6)?
        if(amode == ATA_MODE_LBA28 ||
           amode == ATA_MODE_CHS)
        {
            if(rw == ATA_XFER_READ)
                cmd = (ataInfo.sectorsPerIRQ & 0xFF) > 1 ? ATA_CMD_READ_MULTIPLE : ATA_CMD_READ;
            else
                cmd = (ataInfo.sectorsPerIRQ & 0xFF) > 1 ? ATA_CMD_WRITE_MULTIPLE : ATA_CMD_WRITE;
        }
        else if(amode == ATA_MODE_LBA48)
        {
            if(rw == ATA_XFER_READ)
                cmd = (ataInfo.sectorsPerIRQ & 0xFF) > 1 ? ATA_CMD_READ_MULTIPLE_EXT : ATA_CMD_READ_EXT;
            else
                cmd = (ataInfo.sectorsPerIRQ & 0xFF) > 1 ? ATA_CMD_WRITE_MULTIPLE_EXT : ATA_CMD_WRITE_EXT;
        }
        else
        {
            // Unsupported mode
            kassert(false);
        }

        // Set irq-wait state early to prevent race when reading; i.e. the irq
        // can come between the writeCommand and wairIRQ calls below, and the
        // handler expects ATA_TXS_AWAITING_IRQ...
        txState = ATA_TXS_AWAITING_IRQ;
        prepareTransferBlock(lba, sectors);

        kassert(txState == ATA_TXS_AWAITING_IRQ);

        // Start xfer
        writeCommand(cmd);

        if(rw == ATA_XFER_READ)
        {
            for(int s=0; s < sectors;)
            {
                int j=0;

                // Expect an irq per full DRQ block
                if(j == ataInfo.sectorsPerIRQ)
                {
                    if(waitIRQ(ATA_TIMEOUT_1000_MS*1000) == E_TIMEOUT)
                    {
                        log << "[ATA ERROR] Timed out waiting for DRQ write "
                               "block interrupt" << eol;
                        break;
                    }

                    txState = ATA_TXS_AWAITING_IRQ;
                }

                // Read DRQ block
                for( ; s < sectors && j < (ataInfo.sectorsPerIRQ & 0xFF); j++, s++)
                {
                    if((res = wait(ATA_STATUS_DRQ)) == E_TIMEOUT)
                    {
                        log << "[ATA READ ERROR] Timed out waiting for DRQ" << eol;
                        break;
                    }

                    // Note that this check is retired in ata6 (word 49, bits 0..7)
                    // However, it will return true for new devices because the LBA
                    // bit is there.
                    if(ataInfo.capabilities0 != 0)
                        ins32(channel->iobase + ATA_REG_DATA,
                              buffer + (s*sectorSize), sectorSize/4);
                    else
                        ins16(channel->iobase + ATA_REG_DATA,
                              buffer + (s*sectorSize), sectorSize/2);
                }
            }
        }
        else if(rw == ATA_XFER_WRITE)
        {
            for(int s=0; s < sectors; )
            {
                // For writing, we must await DRQ before writing each *sector
                // block* (i.e 16 sectors per interrupt) and then await IRQ
                // after each block; wait(...) also captures any errors.

                // Set state *before* sending DRQ block; doing it afterwards
                // gives a race condition.
                txState = ATA_TXS_AWAITING_IRQ;

                // Write DRQ block
                int j=0;
                for( ; s < sectors && j < (ataInfo.sectorsPerIRQ & 0xFF); j++, s++)
                {
                    if((res = wait(ATA_STATUS_DRQ)) == E_TIMEOUT)
                    {
                        log << "[ATA WRITE ERROR] Timed out waiting for DRQ" << eol;
                        break;
                    }

                    if(ataInfo.capabilities0 != 0)
                        outs32(channel->iobase + ATA_REG_DATA,
                               buffer + (s*sectorSize), sectorSize/4);
                    else
                        outs16(channel->iobase + ATA_REG_DATA,
                               buffer + (s*sectorSize), sectorSize/2);
                }

                // Await IRQ after each full DRQ block
                if(j == ataInfo.sectorsPerIRQ)
                {
                    if(waitIRQ(ATA_TIMEOUT_1000_MS * 1000) == E_TIMEOUT)
                    {
                        log << "[ATA ERROR] Timed out waiting for DRQ write block interrupt" << eol;
                        res = E_TIMEOUT;
                        break;
                    }

                    txState = ATA_TXS_AWAITING_IRQ;
                }

            }
        }
        else
            kassert(false);
    }
    else
        log << "NOT READY FOR XFER" << eol;

    // Check that we leave in a sane state
    waitReady();
    endTransaction();

    return res;
}

/**
 * Prepare addressing registers according to addressing mode, select addr mode
 * and drive in the DEV register. This can prepare both reading and writing in
 * all modes.
 *
 * @param lba Logical block address, start
 * @param sectors Number of sectors to transfer; must be <1..256>
 */
void ATADriver::prepareTransferBlock(uint64 lba, int sectors)
{
    // todo: return error_t
    kassert(sectors <= maxSectors);

    // Zero sectors actually means 256 sectors; 65K in lba48 mode
    if(sectors == maxSectors)
        sectors = 0;

    uint8 lbaLow=0, lbaMid=0, lbaHi=0, devreg=0;
    uint8 lbaLow_Prev=0, lbaMid_Prev=0, lbaHi_Prev=0;

    if(amode == ATA_MODE_LBA28)
    {
        lbaLow = lba & 0xFF;
        lbaMid = (lba >> 8) & 0xFF;
        lbaHi  = (lba >> 16) & 0xFF;

        // Lower 4 bits of devreg contains LBA[27:24]
        devreg = (lba >> 24) & 0xF;

        // Set lba bit and drive
        devreg |= 0x40;
        devreg |= (drivenum == 0 ? 0xA0 : 0xB0);
    }
    else if(amode == ATA_MODE_LBA48)
    {
        // For lba 48, we need to use FIFO regs
        lbaLow = lba & 0xFF;
        lbaMid = (lba >> 8) & 0xFF;
        lbaHi  = (lba >> 16) & 0xFF;

        lbaLow_Prev = (lba >> 24) & 0xFF;
        lbaMid_Prev = (lba >> 32) & 0xFF;
        lbaHi_Prev  = (lba >> 40) & 0xFF;

        // Set lba bit and drive + set bit 5 and 7 (spec says to fix them so);
        // lower 4 bits are reserved in lba48 per ATAPI-6
        devreg |= 0x40;
        devreg |= (drivenum == 0 ? 0xA0 : 0xB0);
    }
    else
    {
        // The alorithm for convert LBA to CHS is:
        // FIXME: only tested for <= 528 drives; there are claims of <= 8GB CHS
        // drives existing, supporting 256 "heads".
        uint16 cyl = lba / (ataInfo.heads * ataInfo.sectors);
        uint16 tmp = lba % (ataInfo.heads * ataInfo.sectors);
        uint8 head = tmp / ataInfo.sectors;
        uint8 sect = (tmp % ataInfo.sectors) + 1;

        lbaLow = sect;

        // Low cylinder word
        lbaMid = cyl & 0xFF;

        // In really old disk, only the upper two bits are used in high-cyl byte
        // (1024 cyls), while more recents ones use all bits (this providing 65K
        // cyls).
        lbaHi  = (cyl >> 8) & 0xFF;
        devreg = head & 0x0F;

        // Set drive and 101xxxxx pattern for legacy CHS devices
        devreg |= (drivenum == 0 ? 0xA0 : 0xB0);
    }

    // Write high values
    if(amode == ATA_MODE_LBA48)
    {
        out8(channel->iobase+ATA_REG_SECTORCOUNT, (sectors >> 8) & 0xFF);
        out8(channel->iobase+ATA_REG_LBA_LOW, lbaLow_Prev);
        out8(channel->iobase+ATA_REG_LBA_MID, lbaMid_Prev);
        out8(channel->iobase+ATA_REG_LBA_HIGH, lbaHi_Prev);

        // Prev device register content is reservered
        out8(channel->iobase+ATA_REG_DEVSEL, 0);
    }

    // Write low values
    out8(channel->iobase+ATA_REG_SECTORCOUNT, sectors & 0xFF);
    out8(channel->iobase+ATA_REG_LBA_LOW, lbaLow);
    out8(channel->iobase+ATA_REG_LBA_MID, lbaMid);
    out8(channel->iobase+ATA_REG_LBA_HIGH, lbaHi);
    out8(channel->iobase+ATA_REG_DEVSEL, devreg);
}

/**
 * ATA interrupts are per controller; only one drive is active at any given time
 * on the same controller due to the channel lock. However, we may still share
 * interrupts with other devices (e.g. on PCI systems) but that's fine.
 * <p>
 * Also note that, if there are two drives on a channel, we'll get an interrupt
 * even when the *other* driver is operating - hence we must check that we are
 * in fact in "await irq" mode (again, only one drive on the same channel can
 * possibly be in this state due to the channel lock)
 */
int ATADriver::onInterrupt(ContextState* ctx)
{
    if(txState == ATA_TXS_AWAITING_IRQ ||
       /* Because the xfer code may not be able to set AWAITING state before we
          get another irq: */
       txState == ATA_TXS_IRQ_RECEIVED )
    {
        // TODO: someone claimed reading status here to clear irq is a good
        // idea...?
        //uint8_t status = in8(channel->iobase+ATA_REG_STATUS);

        txState = ATA_TXS_IRQ_RECEIVED;

#ifdef APOX_ATA_VERBOSE
//    log << "$";
#endif
    }
    else
    {
        // On certain devices, we may get interrupts even if we don't await
        // on explicitely (such as when entering idle state). What to do?

#ifdef APOX_ATA_VERBOSE
        //log << "\n[ATA] Out-of-band IRQ [vector " << ctx->irqNumber << "], tx state = " << txState << eol;
#endif
    }

    return 0;
}

/**
 * Start channel transaction, sleeplocks if required
 */
void ATADriver::startTransaction()
{
    channel->lock();
    kassert(txState == ATA_TXS_READY);

    txState = ATA_TXS_STARTED;
}

/** Unlocks the controller and the other device can be used */
void ATADriver::endTransaction()
{
    txState = ATA_TXS_READY;
    channel->unlock();
}

/** Wait N microseconds for interrupt */
error_t ATADriver::waitIRQ(int timeout)
{
    uint64 start = Delay::getCPUCycles();
    while(txState != ATA_TXS_IRQ_RECEIVED)
    {
        if (Delay::usSince(start) > (uint64)timeout)
        {
            return E_TIMEOUT;
        }
    }

    return E_OK;
}

/**
 * Wait for the specific status mask to match. Checks for errors.
 *
 * TODO: we should maybe use an aux timer here, rather than spinning - during
 * i/o idling, the CPU should be able to do real work, not spinning around here.
 * However, some of these waits are very small, and the context switch isn't
 * exactly free... where's the threshold?
 *
 * For PIO mode, I'm not sure how much of a problem spin waiting is, and for
 * BM DMA, we'll probably ctx switch (and thus sleep) after issuing the command
 * anyway, or maybe we block on a data completion port or something?
 *
 * @param statusMask Status mask to wait for, such as (ATA_STATUS_DRQ | ATA_STATUS_DRDY)
 * @param keepPendingIRQ Read the alternate status register instead of the
 *        primary status; this preserves the IRQ state
 * @param timeout Microseconds to wait
 * @param sleep If true, sleep 1 ms between each check. It makes sense to set it
 *              to 'true' when waiting for data, 'false' when waiting for status
 *
 * @return E_DEVICE_ERROR, E_TIMEOUT
 */
error_t ATADriver::wait(uint8 statusMask, bool keepPendingIRQ/*=false*/,
                        int timeout /*=ATA_TIMEOUT_US_DEFAULT*/, bool sleep/*=false*/)
{
    // The datasheet says we should wait 400ns before reading the status reg in
    // many cases; we always do it
    Delay::ns(400);

    while (timeout--)
    {
        uint8_t status = in8(channel->iobase+ (keepPendingIRQ ? ATA_REG_ALT_STATUS : ATA_REG_STATUS));

        if(status & ATA_STATUS_ERR)
        {
#ifdef APOX_ATA_VERBOSE
            log << "\n[ATA] Error encountered in wait(...): ";
            log << H(in8(channel->iobase+ATA_REG_ERROR)) << eol;
#endif
            return E_DEVICE_ERROR;
        }

        // Busy flag must be cleared, and drive must be ready for commands
        if (!(status & ATA_STATUS_BSY) &&
            ((status & statusMask) == statusMask))
        {
            break;
        }

        if(sleep)
        {
            timeout -= 1000;
            Thread::sleep(1);
        }
        else
        {
            timeout -= 1;
            Delay::us(1);
        }
    }

#ifdef APOX_ATA_VERBOSE
    if(timeout == 0)
        log << "[ATA] ERROR: wait(...) timed out" << eol;
#endif

    return timeout > 0 ? E_OK : E_TIMEOUT;

}

/**
 * Wait for not-busy condition. Checks for errors.
 *
 * @param timeout Microseconds to wait (microsec-granularity busy looping)
 * @return E_DEVICE_ERROR, E_TIMEOUT
 */
error_t ATADriver::waitNotBusy(int timeout /*=ATA_TIMEOUT_US_DEFAULT*/)
{
    while (timeout--)
    {
        uint8_t status = in8(channel->iobase+ATA_REG_STATUS);

        if(status & ATA_STATUS_ERR)
        {
#ifdef APOX_ATA_VERBOSE
            log << "\n[ATA] Error encountered in waitNotBusy: ";
            log << H(in8(channel->iobase+ATA_REG_ERROR)) << eol;
#endif
            return E_DEVICE_ERROR;
        }

        // Busy flag must be cleared, and drive must be ready for commands
        if (!(status & ATA_STATUS_BSY))
            break;

        Delay::us(1);
    }

#ifdef APOX_ATA_VERBOSE
    if(timeout == 0)
        log << "[ATA] ERROR: waitNotBusy timed out" << eol;
#endif

    return timeout > 0 ? E_OK : E_TIMEOUT;

}

/**
 * Wait for "ready-to-receive commands" state. Checks for errors.
 *
 * @param timeout Microseconds to wait (microsec-granularity busy looping)
 * @param sleep If true, sleep 1 ms between each check. Set to 'true' when waiting for data, 'false' when waiting for status.
 * @return E_DEVICE_ERROR, E_TIMEOUT
 */
error_t ATADriver::waitReady(int timeout /*=ATA_TIMEOUT_US_DEFAULT*/, bool sleep/*=false*/)
{
    while(timeout >= 0)
    {
        uint8_t status = in8(channel->iobase+ATA_REG_STATUS);

        // Errors may or may not be expected (i.e. the probe code expect aborts); in any
        // case we record the last error and returns a generic device error.
        if(status & ATA_STATUS_ERR)
        {
            lastError = in8(channel->iobase+ATA_REG_ERROR);

#ifdef APOX_ATA_VERBOSE
            log << "[ATA ERROR] " << H(lastError) << eol;
#endif
            return E_DEVICE_ERROR;
        }

        // Busy flag must be cleared, and drive must be ready for commands
        if (!(status & ATA_STATUS_BSY) && (status & ATA_STATUS_DRDY))
        {
            break;
        }

        if(sleep)
        {
            timeout -= 1000;
            Thread::sleep(1);
        }
        else
        {
            timeout -= 1;
            Delay::us(1);
        }
    }

#ifdef APOX_ATA_VERBOSE
    if(timeout == 0)
        log << "[ATA] ERROR: waitReady timed out" << eol;
#endif

    return timeout > 0 ? E_OK : E_TIMEOUT;
}

/**
 * Fix-up ATA strings by swapping word bytes
 *
 * @param str String to fix
 * @param len Length of string (since it's not 0 terminated)
 * @param terminate Whether or not to 0 terminate (at len). Doing so may
 *                  overwrite the last char in a long ata string.
 */
void ATADriver::fixATAString(uint8* str, int len, bool terminate/*=false*/)
{
    // Need even-length strings
    kassert(!(len & 1));

    for(int i=len; i > 0; i-=2)
    {
        uint8 tmp = str[i-1];
        str[i-1] = str[i-2];
        str[i-2] = tmp;
    }

    if(terminate)
    {
        str[len-1] = 0;
    }
}
