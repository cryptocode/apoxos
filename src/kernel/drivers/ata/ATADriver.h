/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * ATA device driver
 *
 * \ingroup Devices
 * @{
 */

#ifndef _APOX_ATA_DRIVER_H_
#define _APOX_ATA_DRIVER_H_

#include <kernel/drivers/ata/ATA.h>
#include <kernel/drivers/Device.h>
#include <kernel/concurrency/Mutex.h>
#include <kernel/io/Channel.h>

/**
 * Result of IDENTIFY DEVICE command as specified in the ATA/ATAPI specification
 * (last updated against ATA/ATAPI 6)
 *
 * When C/H/S is 16383/16/63, we got ourselves a large disk.
 *
 * This structure is mostly used initially - all the important information is
 * collected or derived from it and stored in the corresponding ATADriver
 * instance in a convenient form.
 *
 * The fields below indicate word position by number, and whether the words are
 * optional (O), mandatory (M), obsolete (X) or retired (R) per ATAPI-6.
 */
class ATADeviceInfo
{
public:

    /** M 0: Configuration */
    uint16 config;
    /** X 1: Cylinders */
    uint16 cylinders;
    /** O 2: Specific configuration */
    uint16 specificConfig;
    /** X 3: Heads */
    uint16 heads;
    /** R 4-5 */
    uint16 retired0[2];
    /** X 6: Sectors per track */
    uint16 sectors;
    /** O 7-8: Reserved for CompactFlash */
    uint16 compactFlash0[2];
    /** R 9 */
    uint16 retired1;
    /** M 10-19: Serial number */
    uint8 serial[20];
    /** R 20-21: Retired */
    uint16 retired2[2];
    /** X 22: Obsolete */
    uint16 obsolete0;
    /** M 23-26: Firmware Revision (8 ata ascii)*/
    uint8 rev[8];
    /** M 27-46: Model name (40 ata ascii) */
    uint8 model[40];
    /** M 47: [7:0] Max sectors per interrupt; this defines a DRQ block; use SET MULTIPLE to change from default */
    uint16 sectorsPerIRQ;
    /** 48: Reserved */
    uint16 reserved0;
    /** M 49: First capability section (including whether 32-bit xfers can be used) */
    uint16 capabilities0;
    /** M 50: Most of this word is reservered; the bits that aren't are either obsolete or have a fixed value */
    uint16 capabilities1;
    /** 51-52 are obsolete */
    uint16 pio;                  // PIO data transfer cycle timing (0=slow, 1=medium, 2=fast)
    uint16 dma;                  // DMA data transfer cycle timing (0=slow, 1=medium, 2=fast)
    /** 53: Valid fields indicator*/
    uint16 validFields;
    /** 54-58 are obsolete */
    uint16 logcyl;
    uint16 loghead;
    uint16 logspt;
    uint16 cap0;
    uint16 cap1;
    /** 59: Multisector settings */
    uint16 multiSectorSettings;
    /** 60-61: Total number of user addressable sectors */
    uint16 lba28max[2];
    /** 62: Obsolete */
    uint16 dmasingle;
    /** 63: Multiword DMA */
    uint16 dmaMultiword;
    /** 64: [7:0] PIO modes supported */
    uint16 pioModes;
    /** 65: Minimum multiword DMA transfer cycle time per word (nanoseconds) */
    uint16 minmulti;
    /** 66: Recommended multiword DMA transfer cycle time (nanoseconds) */
    uint16 multitime;
    /** 67: Minimum PIO transfer cycle time with no flow control (nanoseconds) */
    uint16 minpio;
    /** 68: Minimum PIO transfer cycle time with IORDY flow control (nanoseconds) */
    uint16 miniodry;
    /** 69-70: Reserved for overlap and queuing */
    uint16 reserved2[2];
    /** 71-74: Reserved */
    uint16 reserved3[4];
    /** 75: [4:0] Queue depth - 1, i.e. 0 => queue depth 1 */
    uint16 queueDepth;
    /** 76-79: Reserved */
    uint16 reserved4[4];
    /** 80: Major version number */
    uint16 majorVersion;
    /** 81: Minor version number */
    uint16 minorVersion;
    /** 82: Command set supported, part 1 */
    uint16 cmdSet1;
    /** 83: Command set supported, part 2 */
    uint16 cmdSet2;
    /** 84: Command set/feature supported extension */
    uint16 cmdSetExt;
    /** 85: Command set/feature enabled, part 1 */
    uint16 cfs_enable_1;
    /** 86: Command set/feature enabled, part 2 */
    uint16 cfs_enable_2;
    /** 87: Command set/feature default */
    uint16 cmdSetDefault;
    /** 88: DMA ultra mode, [13:8] Selected mode, [5:0] Supported mode */
    uint16 dmaultra;
    /** 89: Time required for security erase unit completion */
    uint16 securityEraseTime;
    /** 90: Time required for Enhanced security erase completion */
    uint16 enhancedSecurityEraseTime;
    /** 91: Current APM value */
    uint16 currentAPM;
    /** 92: Master password revision code */
    uint16 masterPwdRevCode;
    /** 93: Hardware reset result */
    uint16 hwResetResult;
    /** 94: Accousting mgmt values */
    uint16 accoustics;
    /** 95-99: Reserved */
    uint16 reserved5[5];
    /** 100-103: Maximum user LBA for 48-bit address feature set */
    uint16 lba48max[4];
    /** 104-126: Reserved */
    uint16 reserved6[23];
    /** 127: Removable Media Status Notification feature set support */
    uint16 removableMediaStatus;
    /** 128: Security status */
    uint16 securityStatus;
    /** 129-159: Vendor specific */
    uint16 vendorSpecific[31];
    /** 160: CFA power mode 1 */
    uint16 cfaPowerMode1;
    /** 161-175: Reserved */
    uint16 reserved7[15];
    /** 176-205: Current media serial number */
    uint16 mediaSerialNumber[30];
    /** 206-254: Reserved */
    uint16 reserved8[49];
    /** 255: [15:8] Checksum, [7:0] Signature */
    uint16 integrityWord;
};


class ATABus;
class ATADriver;

/**
 * Represents an ATA disk in the device tree
 */
class ATADevice : public Device
{
    public:
};

/**
 * The ATA i/o channel interface
 */
class ATAIOChannel : public Channel
{
    public:

        /**
         * C'tor
         *
         * @param driver The driver instance we're providing a Channel on
         */
        ATAIOChannel(ATADriver* driver);

        /** Clone the channel */
        virtual AbstractChannel* clone();

        /* Implements the Channel interface */

        virtual error_t setPosition(fpos_t position);
        virtual ssize_t read(uint8* bytes, size_t length, bool block = true);
        virtual ssize_t write(const uint8* bytes, size_t length, bool block = true);

    private:

        ATADriver* driver;
};

/**
 * TODO: Rename this to ATAInterface... this will be confusing when adding the highlevel Channel interface
 *
 * TODO: Let ATAChannel inherit Port, to make portio safer
 *
 * A channel that controls one or two ATA devices.
 *
 * All access to a channel is serialized, which happends as ATADriverr#startTransaction
 * calls lock() on its ATAChannel. The lock is freed when ATADriver#endTransaction is called.
 *
 * The channel lock is a sleep mutex.
 */
class ATAChannel : public Object
{
    public:

        /** C'tor */
        ATAChannel(ATABus* bus, uint32 iobase, uint8 irqline)
        {
            this->bus = bus;
            this->iobase = iobase;
            this->irqline = irqline;
        }

#if 0
        /** Lock the channel */
        void lock()
        {
            txLock.aquire();
        }

        /** Unlock the channel */
        void unlock()
        {
            txLock.release();
        }
#endif
        /** Channel's IO port base */
        uint32 iobase;

        /** Channel IRQ */
        uint8 irqline;

    private:

        /** The ATA bus */
        ATABus* bus;

        /** Transaction lock */
        //Mutex txLock;
};


// Poor man's cache. Set to 0 to disable caching.
#define ATA_CACHE_SIZE  0

struct ATACache
{
    uint8 data[512];
    uint64 lba;
    // not used yet
    bool dirty;
};


/**
 * Drives a specific device
 */
class ATADriver : public DeviceDriver, public ResourceManager
{
    friend class ATANode;

    public:

    /** Transaction states */
    enum ATATxState
    {
        ATA_TXS_INVALID,
        /** Ready for business */
        ATA_TXS_READY,
        /** A transaction has started */
        ATA_TXS_STARTED,
        /** Awaiting an interrupt */
        ATA_TXS_AWAITING_IRQ,
        /** IRQ processed */
        ATA_TXS_IRQ_RECEIVED,
    };

    /** Addressing modes */
    enum ATAMode
    {
        ATA_MODE_INVALID,
        ATA_MODE_CHS,
        ATA_MODE_LBA28,
        ATA_MODE_LBA48,
    };

    /** Programming modes */
    enum ATAPMode
    {
        ATA_PMODE_INVALID,
        ATA_PMODE_PIO,
        ATA_PMODE_UDMA,
    };

    enum ATARW
    {
        ATA_XFER_READ,
        ATA_XFER_WRITE,
    };

    /* Component interface*/
    public:

        /**
         * Support IIDChannelID
         *
         * TODO: should be able to ask ATADevice the same thing
         */
        virtual Object* getInterface(int IID)
        {
            ATAIOChannel* res = 0;

            if(IID == IIDChannel)
            {
                res = new ATAIOChannel(this);
                res->setResourceManager(this);

                // Allow the channel to work on the entire disk
                res->setCapacity(capacity);
                res->setPosition(0);
            }

            return res;
        }

        /** @see ResourceManager#release */
        virtual error_t release(Object* obj)
        {
            if(obj)
            {
                delete (ATAIOChannel*) obj;
            }

            return E_OK;
        }

        /**
         * Release the interface
         */
        /*virtual void releaseInterface(void* instance)
        {
            if(instance)
            {
                // TODO: need some way of checking that this is a ptr to
                // an object from our getInterface...
                delete (ATAIOChannel*)instance;
            }
        }*/


    /* Public low-level device interface */
    public:

        /**
         * Probes for an ATA(PI) device on the given controller/drivenum slot
         *
         * @param drivenum 0 or 1
         * @returns The driver for the device, or NULL if no device was found
         */
        static ATADriver* initialize(ATAChannel* channel, uint8 drivenum);

        /**
         * Probe for a device
         *
         * @returns True if the device was present, else false
         */
        bool identify();

        /** D'tor */
        virtual ~ATADriver();

        /**
         * Read sectors using the current access method; this is the public block interface; used by higher
         * levels, like the Channel interface.
         */
        error_t readSectors(uint64 lba, int sectors, uint8* buffer);

        /**
         * Write sectors using the current access method
         */
        error_t writeSectors(uint64 lba, int sectors, uint8* buffer);

        /** Returns the number of bytes per sector */
        int getSectorSize();

        /**
         * ATA interrupts are per controller, hence more than one ATADriver instance
         * may receive a single interrupt.
         */
        virtual int onInterrupt(ContextState* ctx);

        /**
         * Locks the controller, selects the device. While a transaction is in flight, any other
         * device on this device's controller can only start a transaction if *this* transaction
         * for an overlapped ATA command
         */
        void startTransaction();

        /** Unlocks the controller and the other device can be used, state becomes ATA_TXS_READY. */
        void endTransaction();

        /** Is packet device? */
        bool getIsPacket();

        /**
         * Get addressing mode
         * @returns ATAMode value
         */
        int getAddressingMode();

        /**
         * Fix-up ATA strings by swapping word bytes
         *
         * @param str String to fix
         * @param len Length of string (since it's not 0 terminated)
         * @param terminate Whether or not to 0 terminate (at len). Doing so may overwrite the last char in a long ata string.
         */
        void fixATAString(uint8* str, int len, bool terminate=false);


        /** Self test */
        void test();

    protected:

        /**
         * Protected c'tor
         *
         * @param portbase ATA channel I/O port base (e.g. 0x1F0 or 0x170)
         * @param drivenum ATA channel drive selector (0 or 1)
         */
        ATADriver(ATAChannel* channel, uint8 drivenum);

        /**
         * Wait for ready state (not busy, and drive ready for commands); uses busy looping.
         *
         * @param timeout Microseconds to wait
         * @param sleep If true, sleep 1 ms between each check. Set to 'true' when waiting for data, 'false' when waiting for status.
         */
        error_t waitReady(int timeout=ATA_TIMEOUT_US_DEFAULT,bool sleep=false);

        /**
         * Wait for the specific status mask to match. Checks for errors.
         *
         * @param statusMask Status mask to wait for, such as (ATA_STATUS_DRQ | ATA_STATUS_DRDY)
         * @param timeout Microseconds to wait (microsec-granularity busy looping)
         * @param sleep If true, sleep 1 ms between each check. Set to 'true' when waiting for data, 'false' when waiting for status.
         */
        error_t wait(uint8 statusMask, bool keepPendingIRQ=false, int timeout=ATA_TIMEOUT_US_DEFAULT, bool sleep=false);

        /**
         * Wait for not-busy condition. Checks for errors.
         *
         * @param timeout Microseconds to wait (microsec-granularity busy looping)
         */
        error_t waitNotBusy(int timeout=ATA_TIMEOUT_US_DEFAULT);

        /** Wait N microseconds for interrupt */
        error_t waitIRQ(int timeout=ATA_TIMEOUT_IRQ_MS*1000);

        /**
         * Setup device info we need later on
         */
        void setup();

        /** Hide details of writing commands so we don't forget to delay 400 ns, etc */
        void writeCommand(int cmd);

        /**
         * Transfer sectors to or from buffer using PIO
         */
        error_t xferPIO(int rw, uint64 lba, int sectors, uint8* buffer);


        /**
         * Resets a packet device. This method has no effect on non-packet devices.
         */
        void resetDevice();

        /**
         * Prepare addressing registers according to addressing mode, select addr mode and drive
         * in the DEV register. This can prepare both reading and writing in all modes.
         *
         * @param lba Logical block address, start
         * @param sectors Number of sectors to transfer
         */
        void prepareTransferBlock(uint64 lba, int sectors);


        /* DEBUG METHODS */

        /**
         * Prints hex + ascii output; 0 shown as blanks
         */
        void debugPrintSectors(const uint8* buf, int sectors);


    private:

        /** Drive on channel [0|1] */
        uint8 drivenum;

        /** ATA channel this device is attached to */
        ATAChannel* channel;

        /** Result of ATA_CMD_IDENTIFY */
        ATADeviceInfo ataInfo;

        /** ATA or ATAPI? */
        bool isPacket;

        /** Addressing mode; CHS, LBA28 or LBA48 */
        ATAMode amode;

        /** Programming mode; PIO or Bus Mastering DMA */
        ATAPMode pmode;

        /** Capacity of the device, in bytes. This is deduced using the (P)IDENTIFY info */
        uint64 capacity;

        /** Universally 512, but who knows what the future brings */
        int sectorSize;

        /** Max number of sectors per xfer; 256 for CHS/LBA28; 65K for LBA48 */
        int maxSectors;

        /** Set to true when an interrupt has occured */
        volatile bool irqReceived;

        /** Last value read from the ATA error register */
        uint8 lastError;

        /** What state are we in? Initialized to 'ATA_TXS_READY' */
        volatile ATATxState txState;

        ATACache cache[ATA_CACHE_SIZE];
        long cacheHits, cacheMisses;
};

/* Inlines */

inline bool ATADriver::getIsPacket()
{
    return isPacket;
}

inline int ATADriver::getAddressingMode()
{
    return amode;
}

inline int ATADriver::getSectorSize()
{
    return sectorSize;
}

/**
 * A namespace node for ATA devices; overrides getChannel so that each
 * request for channel results in a new instance.
 */
class ATANode : public Node
{
    public:

        /** C'tor */
        ATANode(const String& name, ATADriver* driver) : Node(name)
        {
            this->driver = driver;
        }

    private:

        ATADriver* driver;
};


#endif // _APOX_ATA_DRIVER_H

/**@}*/
