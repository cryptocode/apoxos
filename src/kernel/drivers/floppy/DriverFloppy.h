/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#ifndef _APOX_APOX_DRIVER_SERIAL_H_
#define _APOX_APOX_DRIVER_SERIAL_H_

#include <kernel/libc/std.h>
#include <kernel/init/context.h>
#include <kernel/adt/Queue.h>
#include <kernel/io/Channel.h>
#include <kernel/drivers/Device.h>

/**
 * \name Floppy error codes
 * @{
 */
#define FDC_ERR_NO_ERROR    0
#define FDC_ERR_NO_RESET    1
/**@}*/

/** Max number of supported drives */
#define FLOPPY_MAX_DRIVES           2

/** Physical address of floppy parameter table */
#define FLOPPY_PARAMETER_ADDRESS    0x000FEFC7

/** How long to wait before actually stopping the motor */
#define FDC_MOTOR_STOP_DELAY_MS     5000

#define FDC_PRIMARY_BASE            0x3F0
#define FDC_SECONDARY_BASE          0x370

#define FLOPPY_NUMHEADS             2
#define FLOPPY_NUMTRACKS_DEFAULT    80

// FDC registers, relative to controller base port
#define FDC_REG_DOR     0x2   /* Digital Output Register */

/** Main Status Register. Monitored to control reads and writes */
#define FDC_REG_MSR     0x4

/** Data Rate Select */
#define FDC_REG_DRS     0x4

/** Data register */
#define FDC_REG_DATA    0x5

/** Digital Input Register (in) */
#define FDC_REG_DIR     0x7

/** Configuration Control Register (out) */
#define FDC_REG_CCR     0x7

#define FDC_STATUS0     0x01
#define FDC_STATUS1     0x02
#define FDC_STATUS2     0x03
#define FDC_STATUS3     0x00  // for CMD_SENSE_DRIVE_STATUS

/**
 * \name MSR flags
 * @{
 */
#define FDC_MSR_BIT_ACTIVE_A    0
#define FDC_MSR_BIT_ACTIVE_B    1
#define FDC_MSR_BIT_ACTIVE_C    2
#define FDC_MSR_BIT_ACTIVE_D    3
#define FDC_MSR_BIT_BUSY        4
#define FDC_MSR_BIT_NDMA        5

/** Direction (reading from CPU or writing to CPU) */
#define FDC_MSR_BIT_DIO         6

/** Main request status (1=data register ready, 0=data register not ready) */
#define FDC_MSR_BIT_MRQ         7

/**@} MSR flags */

// ST0 flags (interrupt codes, seek end, etc)
#define FDC_ST0_IC1             7
#define FDC_ST0_IC0             6
#define FDC_ST0_SEEK_END        5
#define FDC_ST0_UNIT_CHECK      4
#define FDC_ST0_DRIVE_NOTREADY  3
#define FDC_ST0_ACTIVE_HEAD     2
#define FDC_ST0_CURRENTDRIVE_1  1
#define FDC_ST0_CURRENTDRIVE_0  0

// REG_DOR values
#define FDC_DOR_VAL_RESET           4
#define FDC_DOR_VAL_DMAGATE         8
#define FDC_DOR_VAL_MOTOR_A         16
#define FDC_DOR_VAL_MOTOR_B         32

// REG DIR values
#define FDC_DIR_FLAG_MEDIACHANGE    7

// Retry-count for accessing FDC registers
#define FDC_RETRIES_MAX         20000

/**
 * \name Floppy commands
 * \brief Initialize internal drive timers. There is no result phase.
 * \note Instead of, say, three read sector commands, there should be
 * only one with separate MT, MFM modifiers -> much cleaner.
 * @{
 */
#define FDC_CMD_SPECIFY                 0x03
#define FDC_CMD_SENSE_DRIVE_STATUS      0x04
#define FDC_CMD_WRITE_SECTORS           0xC5  /* write data (+ MT,MFM) */
#define FDC_CMD_WRITE_DELETE_SECTOR     0xC9
#define FDC_CMD_READ_SECTORS            0xC6  /* read data (+ MT,MFM) */
#define FDC_CMD_READ_SECTORS_EX         0x66  /* read data */
#define FDC_CMD_READ_SECTORS_SKIPMODE   0xE6  /* read data (+ MT,MFM,SK) */
#define FDC_CMD_READ_DELETE_SECTOR      0xCC
#define FDC_CMD_READ_TRACK              0x42
#define FDC_CMD_RECALIBRATE             0x07  /* recalibrate */
#define FDC_CMD_SENSE_INTERRUPT         0x08  /* sense interrupt status */
#define FDC_CMD_FORMAT                  0x4D  /* format track (+ MFM) */
#define FDC_CMD_SEEK                    0x0F  /* seek track */
#define FDC_CMD_VERSION                 0x10  /* FDC version */
#define FDC_CMD_READ_SECTORID           0x4A
/**@}*/

/** Holds the result from an FDC command */
struct FloppyCmdResult
{
    uint8_t  ST0;
    uint8_t  ST1;
    uint8_t  ST2;
    uint8_t  cylinder;
    uint8_t  head;
    uint8_t  sector;
    uint8_t  sectorSize;
};

typedef struct FloppyCmdResult* FloppyCmdResultPtr;

/**
 * BIOS Area provided floppy parameter table
 *
 * The table is copied into a FloppyParameters instance at boot time from
 * the BIOS data area.
 *
 * I trusted The Undocumented PC 2'nd Edition on the details.
 */
struct FloppyParameters
{
    /** \brief Step rate in high nibble,  head unload time in lower */
    uint8 SRT_HUT;

    /** Head load time in 7..1, NO-DMA indicator in first bit */
    uint8 HUT_NDMA;

    /** \brief Motor delay off time (in 54 ms units!) */
    uint8 motorDelayOff;

    /** \brief Bytes per sector (0=128, 1=256, 2=512 (usual value), etc )*/
    uint8 bytesPerSectorCode;

    /** \brief Sectors per track */
    uint8  sectorsPerTrack;

    /** \brief GAP3 lengths (usually 27) */
    uint8  gapLength;

    /** \brief Always 0xFF */
    uint8  dataLength;

    /** \brief GAP length when formatting */
    uint8  formatGapLength;
    uint8  filler;

    /** \brief Head settle time (in milliseconds) */
    uint8  HST;

    /** \brief Motor startup time (in 1/8 second units) */
    uint8  MST;
};

/** Floppy transfer mode */
enum FloppyMode
{
    FM_READ,
    FM_WRITE
};

class DriverFloppy;

/**
 * Implements the Channel interface to floppy drives
 */
class FloppyChannel : public Channel
{
    public:

        /**
         * C'tor
         */
        FloppyChannel(DriverFloppy* dev);

        /** Clone the channel */
        virtual AbstractChannel* clone();

        /** @see Channel#setPosition */
        virtual error_t setPosition(fpos_t position);

        /** @see Channel#read */
        virtual ssize_t read(uint8* bytes, size_t length, bool block=true);

        /** @see Channel#write */
        virtual ssize_t write(const uint8* bytes, size_t length, bool block=true);

    private:

        DriverFloppy* dev;
};

/**
 * A floppy device (up to two instances is supported)
 */
class DriverFloppy : public DeviceDriver, public ResourceManager
{
    public:

        /**
         * C'tor
         */
        DriverFloppy(FloppyParameters* fdcParams) : DeviceDriver(null)
        {
            this->fdcParams = fdcParams;
            channel = new FloppyChannel(this);
            channel->setResourceManager(this);

            lastError = FDC_ERR_NO_ERROR;

            portBase = FDC_PRIMARY_BASE;
            tracks = FLOPPY_NUMTRACKS_DEFAULT;

            // Compute sector count
            sectorCount = tracks * FLOPPY_NUMHEADS * fdcParams->sectorsPerTrack;

            // From bytes-per-sector code to actual byte count (which is a
            // powers-of-two table starting at 128)
            actualBytesPerSector = (128 << fdcParams->bytesPerSectorCode);

            //position = 0;
            recalibrated = false;
            motorOn = false;
            mediaChanged = false;
        }

        /**
         * Implements Component#queryInterface
         */
        virtual Object* getInterface(int IID)
        {
            Object* res = null;

            if(IID == IIDChannel)
            {
                AbstractChannel* clone = channel->clone();
                clone->setPosition(0);
                res = clone;
            }

            return res;
        }

        /** @see ResourceManager#release */
        error_t release(Object* obj)
        {
            if(obj != null)
            {
                delete (Channel*)obj;
            }

            return E_OK;
        }

        /**
         * @see DeviceDriver#onInterrupt
         */
        virtual int onInterrupt(ContextState* ctx);

        static void initialize();

        /** Queue of user read/write/format/etc requests, initiated by the block device API */
        QueuePtr requests;

        /** Floppy parameters */
        struct FloppyParameters* fdcParams;

        /** Result of last command */
        struct FloppyCmdResult cmdResult;

        /** Number of tracks per side */
        int tracks;

        /** Bytes per sector (the value in fdcParams is table index where 2 means 512, etc) */
        uint16_t actualBytesPerSector;

        /**
         * Total per-cylinder sector count (e.g. 18 sectors per cylinder * 80 tracks * 2 heads = 2880 for 1.44 MB)
         * \note Sectors per track is half of this (i.e. 9 when there's 18 sectors per cylinder)
         */
        uint16_t sectorCount;

        /** 0x3F0 if primary, 0x370 if secondary */
        int portBase;

        /** Drive number (0..3) */
        int driveNumber;

        /** Last error, or 0 if no error. Initialized to 0 whenever a command is executed  */
        int lastError;

        /**
         * \brief Position (used by channel interface). Initialized to 0.
         * \invariant 0 <= position < capasity)
         */
        //int position;

        /** True if the motor is on */
        volatile bool motorOn;

        /** True if the drive has been recalibrated */
        bool recalibrated;

        /** True if media has changed since the last command */
        bool mediaChanged;

    private:

        FloppyChannel* channel;
};

/** \brief Called by the init-subsystem sometime before activating paging */
void copyFloppyParams();

/** \brief Get floppy device handle. If there is no floppy at the given index, NULL is returned. */
DriverFloppy* getFloppy(int index);

/** \brief Start floppy motor */
void floppyStartMotor(DriverFloppy* dev);

/** \brief Stop floppy motor */
void floppyStopMotor(DriverFloppy* dev);

/** \brief Put into known state, including recalibration */
void floppyReset(DriverFloppy* dev);

/** \brief Recalibrates the drive */
bool floppyRecalibrate(DriverFloppy* dev);

/** \brief Read a byte from the data register. Returns -1 if the register couldn't be read (usually timeout) */
int floppyGetByte(DriverFloppy* dev);

/** \brief Write the value to the data register. Returns TRUE if ok, else FALSE */
bool floppyPutByte(DriverFloppy* dev, uint8_t val);

/**
 * \brief Returns TRUE if media has changed since last command.
 * \note dev->mediaChange is updated
 */
bool floppyMediaChanged(DriverFloppy* dev);

/**
 * \brief Get command result
 * \note dev->cmdResult is updated
 * \return flase on timeout, else true
 */
bool floppyGetResult(DriverFloppy* dev);

/** \brief Waits until data is read. Returns FALSE if retries are exhausted */
bool floppyWaitDataReady(DriverFloppy* dev);

/** \brief Reset interrupt flag and send command */
bool floppySendCommand(DriverFloppy* dev, uint8_t command);

/**
 * \brief From linear block address to CHS (cylinder, head, sector)
 *
 * \param lba LBA address 0 .. heads * tracks * (sectorsPerTrack-1)
 */
void floppyGetCHS(DriverFloppy* dev, int lba, int* sector, int* cylinder, int* head);

/** \brief Seek using LBA addressing */
bool floppySeek(DriverFloppy* dev, int lba);

/** \brief Seek using CH addressing */
bool floppySeekEx(DriverFloppy* dev, int cylinder, int head);

/** \brief Returns true if the motor is running (the FDC is queried) */
bool floppyIsMotorOn(DriverFloppy* dev);

/**
 * \brief Read or write sectors
 *
 * \param buffer Buffer to transfer to or from. The size must be a multiple of the sector size
 * \param lba Logical block address (0..N)
 * \param numSectors Number of sectors
 * \param mode Whether to read or write sectors
 */
void floppyTransfer(DriverFloppy* dev, void* buffer, int lba, enum FloppyMode mode);

/** \brief Read sectors into buffer, using linear sector addressing */
void floppyReadSector(DriverFloppy* dev, void* buffer, int lba);

/** \brief Write sectors from buffer, using linear sector addressing */
void floppyWriteSector(DriverFloppy* dev, void* buffer, int lba);

/** \brief Format floppy disk */
void floppyFormat(DriverFloppy* dev);

#endif
