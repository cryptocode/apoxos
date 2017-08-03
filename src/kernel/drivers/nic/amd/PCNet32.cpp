/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

/**
 * Primary reference is the AMD PCnet-PCI II data sheet and the AMD document
 * 'PCnet Familiy Software Design Considerations'. Electronic versions are
 * available in apoxdoc/nic/amd.
 *
 * This implements a driver for the PCnet chipset family from AMD.
 *
 * Overview
 * --------
 * The PCnet-PCI II chip is interfaced using three kinds of registers; namely the
 * PCI config registers, CSR (Control and Status Registers) and BCR (Bus Control Registers).
 *
 * For CSR and BCR access, the wanted register slot is written to the Register Address Port
 * (RAP); this latches the address until the next write to RAP. The actual data is then
 * read and written via the Register Data Port (RDP) for CSR registers, or BCR Data Port
 * for BCR registers. Note that RAP is used for latching addresses for both type of
 * registers (not PCI regs which has its own standardized access method; use Apox's PCIConfig)
 *
 * Alignment requirements
 * ----------------------
 * This driver operates in 32-bit mode. The initialization block must be 4-byte aligned,
 * and the ring descriptors must be 16-byte aligned.
 *
 * Chip support
 * ------------
 *
 * Non-PCI controllers in the PCnet family are not supported, i.e:
 *
 *    Lance, Am7990
 *    C-Lance, Am79C90
 *    ILACC, Am79C900
 *    PCnet-ISA
 *
 * PCI controllers are supported, i.e:
 *
 *    Am79C970, PCnet-PCI Single-Chip Ethernet Controller for PCI
 *    Am79C970A, PCnet-PCI II Single-Chip Full-Duplex Ethernet Controller (reference card)
 *
 * Should work, but not tested:
 *
 *    Am79C971/2/3/4/5/6
 *    Am79C978 (Home edition)
 *
 * Issues
 * ------
 * Switching promiscuous mode doesn't work properly (at least in VMWare)
 */

#include <kernel/drivers/nic/amd/PCNet32.h>
#include <kernel/drivers/nic/amd/PCNet32Regs.h>
#include <kernel/drivers/nic/amd/PCNet32Descriptor.h>
#include <kernel/io/Channel.h>
#include <kernel/io/net/ARP.h>
#include <kernel/io/net/IP.h>
#include <kernel/init/irq.h>
#include <kernel/threads/Thread.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/util/Hexdump.h>
#include <kernel/util/Math.h>
#include <kernel/New.h>
#include <kernel/vmm/vmm.h>
#include <kernel/drivers/bus/pci/PCIDatabase.h>
#include <arch/drivers/bus/pci/PCIConfig.h>
#include <arch/Delay.h>

namespace PCNet32
{
    /** Debug noise */
    #define NOISE                   0

    /** Signals reset to rx/tx threads */
    #define SIG_RESET               1

    /** Signals thread exit to rx/tx threads */
    #define SIG_THREAD_EXIT         2

    /** PCI command reg; read as 16-bit */
    #define PCI_REG_CMD             0x04

    /** PCI byte offset of I/O BAR; read as 32-bit */
    #define PCI_REG_IOBAR           0x10

    /** PCI byte offset of MEM BAR; read as 32-bit */
    #define PCI_REG_MEMBAR1         0x14

    /**
     * Initialization block; we use the 32-bit version so BCR20, bit 8
     * (SSIZE) must be set.
     */
    struct InitializationBlock
    {
        /** Loaded into CSR15, mode register */
        uint16 mode;

        /**
         * Entries in rx ring; only upper 4 bits are used and these are
         * log2(ENTRIES)
         */
        uint8 rxEntries;

        /**
         * Entries in tx ring; only upper 4 bits are used and these are
         * log2(ENTRIES)
         */
        uint8 txEntries;

        /** MAC address */
        uint8 mac[ETH_MAC_BYTES];

        uint16 reserved;

        /** Logical address filter, low */
        uint32 filterLow;

        /** Logical address filter, high */
        uint32 filterHigh;

        /** 32-bit physical rx ring */
        uint32 rxRingBase;

        /** 32-bit physical tx ring */
        uint32 txRingBase;
    };


/**
 * C'tor
 */
PCNet32Driver::PCNet32Driver() :
    macadr(mac, ETH_MAC_BYTES), receiver(this), sender(this)
{
    isPromiscuous = false;
    setMTU(1500);

    // Hack to handle several nics
    static int count = 0;
    if(count == 0)
    {
        // TODO: just testing; should be assigned from outside
        getIPAddresses().add(new IPAddress("10.0.0.120"));
        getIPAddresses().add(new IPAddress("10.0.0.98"));

        count++;
    }
    else
    {
        getIPAddresses().add(new IPAddress("10.0.0.97"));
        getIPAddresses().add(new IPAddress("10.0.0.96"));
    }
}

/** D'tor */
PCNet32Driver::~PCNet32Driver()
{
}

/** Write to a bus control register */
void PCNet32Driver::writeBcr32(int reg, uint32 data)
{
    // Latch and write
    out32(REG_RAP, reg);
    out32(REG_BDP, data);
}

/** Read from a bus control register */
uint32 PCNet32Driver::readBcr32(int reg)
{
    // Latch and read
    out32(REG_RAP, reg);
    return in32(REG_BDP);
}

/** Write to a control status register */
void PCNet32Driver::writeCsr32(int reg, uint32 data)
{
    // Latch and write
    out32(REG_RAP, reg);
    out32(REG_RDP, data);
}

/** Read from a control status register */
uint32 PCNet32Driver::readCsr32(int reg)
{
    // Latch and read
    out32(REG_RAP, reg);
    return in32(REG_RDP);
}

/**
 * The purpose of the read thread is to:
 *
 *  1) Reduce interrupt latency by doing as little as possible (just a wake-up)
 *  2) Do frame/packet handling in thread context so we can sleep, etc
 *
 * @see Threadable#run */
void PCNet32Driver::Receiver::run(void* arg)
{
    for(;;)
    {
        // Wait 'till someone calls notify() on us
        wait();

        // The chip was stopped for restart and the notify count might have been > 0
        // so we have to reset the wait count.
        if(getSignal() == SIG_RESET)
        {
            setSignal(0);
        }
        else if(getSignal() == SIG_THREAD_EXIT)
        {
            break;
        }
        else
        {
            // Notify the driver that there are packets waiting in the rx buffer
            driver->onPacketsAvailable();
        }
    }
}

/** @see Threadable#run */
void PCNet32Driver::Sender::run(void* arg)
{
    for(;;)
    {
        // Get get notify()'ed whenever there is something in the send queue
        wait();

        if(getSignal() == SIG_RESET)
        {
            setSignal(0);
        }
        else if(getSignal() == SIG_THREAD_EXIT)
        {
            break;
        }
        else
        {
            driver->onSendData();
        }
    }
}

/**
 * Get (and if necessary create) a device driver instance
 */
PCNet32Driver* PCNet32Driver::getInstance(PCIDevice* dev)
{
    if(dev->getDriver() == null)
    {
        PCNet32Driver* driver = new PCNet32Driver();
        driver->setDevice(dev);
        dev->setDriver(driver);

        // Register ourselves; we'll automatically be put in the NIC registry.
        String name(DRIVERNAME_PCNET32);
        static int instanceCounter = 0;
        name << "#" << instanceCounter;
        instanceCounter++;

        dev->setName(name);
        DeviceManager::addDevice(dev);

        out << "Created PCNet32Driver, IRQ " << dev->getIrqLine() << ", Dev: "
            << dev->deviceNum << ", FnID: " << dev->functionNum << eol;

        // Create receiver thread; it immediatly puts itself in a wait condition
        // and is woken up via notify() as packets arrive.
        Thread::create(&driver->receiver);

        if(driver->setStatus(DEVSTATUS_RUNNING) != E_OK)
        {
            out << "Couldn't start NIC" << eol;
        }
    }

    return (PCNet32Driver*)dev->getDriver();
}

/**
 * Change device status
 *
 * @see DeviceDriver::setStatus
 */
error_t PCNet32Driver::setStatus(DeviceStatus status)
{
    error_t res = E_OK;

    // TODO: there is a separate suspend option in CSR0 we could use for suspension
    if(status == DEVSTATUS_STOPPED || status == DEVSTATUS_SUSPENDED)
    {
        receiver.setSignal(SIG_RESET);

        // Stop the chip so it doesn't generate any interrupts on us...
        writeCsr32(REG_CSR_STATUS, 1<<CSR_STATUS_STOP);

        // Mask all interrupts and unregister the irq handler
        uninstallIRQHandler(getDevice()->getIrqLine(), this);

        out << "[NIC] Device stopped" << eol;
    }
    else if(status == DEVSTATUS_RUNNING)
    {
        // Register irq handler
        installIRQHandler(getDevice()->getIrqLine(), this);

        // TODO: Must read from config whether to start in promiscious mode or not
        res = prepare();

        if(res == E_OK)
        {
            out << "[NIC] Started ";
        }
    }
    else
    {
        kassert(false);
    }

    // Call default impl' before returning to update some internal state
    DeviceDriver::setStatus(status);

    return res;
}

/**
 * Reset the chip
 *
 * @returns SYSERR_TIMEOUT
 */
error_t PCNet32Driver::reset()
{
    writeCsr32(REG_CSR_STATUS, 1 << CSR_STATUS_STOP);

    // This makes resetting work in both 16 and 32 bit I/O mode
    in16(REG_RESET);
    in32(REG_RESET);

    // TODO: wait 'til ready (poll LED?)
    Thread::sleep(1000);

    // Set DWIO (32-bit) access mode. Any DWIO write to RDP will do and
    // writing a 0 to CSR0 is safe.
    writeCsr32(0, 0);

    return E_OK;
}

/**
 * Prepares the chip for online operation. Does everything but setting up
 * and enabling IRQ
 */
error_t PCNet32Driver::prepare()
{
    error_t res = E_OK;

    //
    // Driver init
    //
    bytesReceived = 0;
    txHead = 0;
    txTail = 0;
    rxDescIdx = 0;

    //
    // PCI initialization
    //
    PCIDevice* dev = (PCIDevice*)getDevice();

    if(dev->getIrqLine() == 0 || dev->getIrqLine() == 0xFF)
    {
        out << "[NIC " << dev->deviceInfo->ChipDesc << "] No IRQ assignement\n";
    }

    // Get I/O mapped register space. TODO: use memory mapped instead
    uint32 iobase;
    PCIConfig::read(dev->getBus()->getBusNumber(), dev->deviceNum,
                    dev->functionNum, PCI_REG_IOBAR/4 , 4, &iobase);

    // Lower two bits are off limits, per PCI spec
    iobase &= ~3;

    // Make the PortIO methods work relative to the NIC's io base
    setIOBase(iobase);
    out << "[PCI] I/O base: " << H(iobase) << eol;

    // Get memory IO base (PCInet uses PCI config space 32-bit offset 5 for this)
    // TODO: we must set this value (physical address), then set MEMEN in the PCI command register
    uint32 membase;
    PCIConfig::read(dev->getBus()->getBusNumber(), dev->deviceNum,
                    dev->functionNum, PCI_REG_MEMBAR1/4 , 4, &membase);

    out << "[PCI] Memory mapped I/O base: " << H(membase) << eol;

    // Get PCI command register...
    uint32 cmdPciReg;
    PCIConfig::read(dev->getBus()->getBusNumber(), dev->deviceNum,
                    dev->functionNum, PCI_REG_CMD/2 , 2, &cmdPciReg);

    // ...and make sure bus mastering and IO access is enabled by setting (BMEM | IOEN)
    cmdPciReg |= 0x05;
    PCIConfig::write(dev->getBus()->getBusNumber(), dev->deviceNum,
                     dev->functionNum, PCI_REG_CMD/2 , 2, cmdPciReg);

    //
    // Chip initialization
    //

    res = reset();
    if(res != E_OK)
        return res;

    // Set SW style to PCNet-PCI mode (2). This means that the software
    // structures, such as the initialization block, is assumed to be 32-bit
    // wide. It also affects the interpretation of a few CSR registers.
    uint32 swstyle = 2;
    writeBcr32(REG_BCR_SWSTYLE, swstyle);

    // Verify that swstyle was set
    if(readBcr32(REG_BCR_SWSTYLE) & (1<<8))
    {
        out << "32-bit SSIZE32 OK: " << H(readBcr32(REG_BCR_SWSTYLE)) << eol;
        out << "BURST AND CONTROL: " << H(readBcr32(REG_BCR_BURST_AND_CONTROL)) << eol;
        out << "CSR STATUS: " << H(readCsr32(REG_CSR_STATUS)) << eol;
    }
    else
    {
        out << "32-bit SSIZE32 NOT OK: " << H(readBcr32(REG_BCR_SWSTYLE)) << eol;
    }

    // Make sure DWIO is set
    uint32 bcr18 = readBcr32(REG_BCR_BURST_AND_CONTROL) | (1 << 7);
    writeBcr32(REG_BCR_BURST_AND_CONTROL, bcr18);

    // Enable full duplex
    writeBcr32(REG_BCR_FD, BCR_FD_FDEN);

    // These flags are cleared by the reset. APAD_XMT makes sure padding is
    // added to frames less than 64 bytes and that frame checksums are
    // generated. Setting the DMAPLUS bit is required for PCI devices.
    writeCsr32(REG_CSR_TEST_FEATURES, (1 << CSR4_DMAPLUS) | (1 << CSR4_APAD_XMT));

    // Clear all indicator bits by writing a one to each bit (upper 16 bits are
    // reserved)
    writeCsr32(REG_CSR_EXT_CTRL,
            1 << CSR5_INTE_LAST_TX |
            1 << CSR5_INTE_SYS |
            1 << CSR5_INTE_SLEEP_DONE |
            1 << CSR5_INTE_EXTD_INT |
            1 << CSR5_INTE_MAGIC_PACKET);


    // Fast suspend mode (not supported in ...70/VMWare) by setting FASTSPNDE
    writeCsr32(REG_CSR6_EXT_CTRL2,
               (readCsr32(REG_CSR6_EXT_CTRL2) & 0xFFFF) | (1 << 15));

    // Get MAC address from the first 6 bytes of the Address PROM
    out << "MAC address: ";
    for(int i=0; i < ETH_MAC_BYTES; i++)
    {
        mac[i] = in8(i);

        if(i > 0)
            out << ":";

        out << H(in8(REG_ADDRESS_PROM + i), false, 2);
    }
    out << eol;

    //
    // Set up memory arena for the device, using physically contiguous memory so
    // that converting between virtual and physical addresses is fast and easy.
    //
    // The layout of the arena is as illustrated below.
    //
    // RxDescriptor 1
    // ...
    // RxDescriptor N
    // Receive buffer for RxDescriptor 1
    // ...
    // Receive buffer for RxDescriptor N

    // TxDescriptor 1
    // ...
    // TxDescriptor N
    // Transmit buffer for TxDescriptor 1
    // ...
    // Transmit buffer for TxDescriptor N
    //
    // INIT BLOCK
    //
    size_t arenasize =
        NUM_RXDESC*sizeof(RxDescriptor) +
        NUM_RXDESC*RX_BUF_LEN +
        NUM_TXDESC*sizeof(TxDescriptor) +
        NUM_TXDESC*TX_BUF_LEN +
        sizeof(InitializationBlock);

    paddr pa; vaddr va;
    assert(VMM::allocMappedRegion(OUT va, OUT pa,
                                     Math::divup(arenasize, VM_PAGE_SIZE)) == E_OK);

    vaddr arena = va;
    physDelta = va - pa;

    printf("ARENA: %x, phys: %x, phys delta: %x\n", arena, pa, physDelta);

    // The Initalization Procedure

    //
    // Set rx and tx buffer for each descriptor in each ring
    // Note: prepare() may be called many times; we need to dealloc existing buffers
    //

    // Rx
    rxRing = new ((void*)arena) RxDescriptor[NUM_RXDESC];
    for(int i=0; i < NUM_RXDESC; i++)
    {
        rxRing[i].initialize(this);
        //rxRing[i].dumpInfo();
    }

    arena += NUM_RXDESC*sizeof(RxDescriptor) + NUM_RXDESC*RX_BUF_LEN;

    // Tx
    txRing = new ((void*)arena) TxDescriptor[NUM_TXDESC];
    for(int i=0; i < NUM_TXDESC; i++)
    {
        txRing[i].initialize(this);
        //txRing[i].dumpInfo();
    }

    arena += NUM_TXDESC*sizeof(TxDescriptor) + NUM_TXDESC*TX_BUF_LEN;

    // Set init block
    InitializationBlock* init = new((void*)arena)InitializationBlock();
    init->mode = 0;
    init->rxEntries = NUM_RXDESC_LOG2 << 4;
    init->txEntries = NUM_TXDESC_LOG2 << 4;
    init->rxRingBase = (uint32) toPhys(rxRing);
    init->txRingBase = (uint32) toPhys(txRing);

    // Copy mac address into init block
    for(int i=0; i < ETH_MAC_BYTES; i++)
    {
        init->mac[i] = mac[i];
    }

    // Logical address filter; set to all 1's
    init->filterLow = (uint32)~0;
    init->filterHigh= (uint32)~0;

    paddr initPhys = (paddr)toPhys(init);
    writeCsr32(REG_CSR_IADR_LOW, (initPhys) & 0xFFFF);
    writeCsr32(REG_CSR_IADR_HI, ((initPhys) >> 16) & 0xFFFF);

    Thread::sleep(10);

    // Init/start and enable controller interrupts
    writeCsr32(REG_CSR_STATUS, 1<<CSR_STATUS_INIT | 1<<CSR_STATUS_IENA);

    return res;
}

/** Send a packet */
error_t PCNet32Driver::sendPacket(uint8* _packet)
{
    error_t res = E_OK;

    return res;
}

/**
 * Test interface
 */
error_t PCNet32Driver::test(int param)
{
    if(param == 0)
    {
        out << "   tnic1: Dump first 1K of the RX buffer" << eol;
        out << "   tnic2: Send ARP packet to broadcast fixed IP " << eol;
        out << "   tnic3: Restart NIC " << eol;
        out << "   tnic4: Suspend NIC " << eol;
        out << "   tnic5: ARP request " << eol;
        out << "   tnic6: Promiscuous mode off " << eol;
        out << "   tnic7: Promiscuous mode on " << eol;
        out << eol;
    }
    if(param == 1)
    {
        out << "[NIC] RECEIVEBUFFER:\n-------------------------------------" << eol;

        // Dump the first 1K
        out << "NO IMPL" << eol;

        out << "----------------------------------------------" << eol;
    }
    else if(param == 2)
    {
        if(getStatus() == DEVSTATUS_RUNNING)
        {
            // Send N packets
            for(int i=0; i< 5; i++)
            {
                sendPacket(null);
            }
        }
        else
        {
            out << "Sorry dude, the NIC is not running; try tnic3 first" << eol;
        }
    }
    else if(param == 3)
    {
        out << "[NIC] Restarting... " << eos;
        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);
        out << "DONE" << eol;
    }
    else if(param == 4)
    {
        setStatus(DEVSTATUS_SUSPENDED);
        out << "SUSPENDED. Use tnic3 to start again" << eol;
    }
    else if(param == 5)
    {
        String* val = new String("10.0.0.6");
        kassert(val != null);

        IPAddress ip(val);
        uint8 hw[ETH_MAC_BYTES];

        if(ArpManager::getInstance(this)->lookup(ip, hw) == E_TIMEOUT)
        {
            out << "Timed out while waiting for ARP reply" << eol;
        }
        else
        {
            out << "ARP reply: " << eol;
            for(int i=0; i < ETH_MAC_BYTES; i++)
            {
                out << H(hw[i], false) << eos;
            }
            out << eol;
        }
    }
    else if(param == 6)
    {
        isPromiscuous = false;
        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);
        return E_OK;

        // Upper 16 bits are undefined
        uint32 ext = readCsr32(REG_CSR_EXT_CTRL) & 0xCFFF;
        writeCsr32(REG_CSR_EXT_CTRL, ext | 1);

        Delay::ms(10);

        // We must poll the read-version of the suspend bit
        for(int i=0; i < 10000; i++)
        {
            uint32 ext = readCsr32(REG_CSR_EXT_CTRL) & 0xCFFF;
            if(ext & 1)
            {
                out << "Chip suspended successfully" << eol;
                break;
            }
            Delay::ms(1);
        }

        // Disable promiscious mode (bit 15)
        uint32 mode = readCsr32(REG_CSR_MODE) & 0xFFFF;
        writeCsr32(REG_CSR_MODE, mode & ~(1 << 15));

        // Clear suspend bit to resume operation
        writeCsr32(REG_CSR_EXT_CTRL, ext);

        // After suspend, start and enable controller interrupts
        writeCsr32(REG_CSR_STATUS, 1<<CSR_STATUS_START | 1<<CSR_STATUS_IENA);
    }
    else if(param == 7)
    {
        isPromiscuous = true;
        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);
        return E_OK;

        // Upper 16 bits and bit 12 and 13 are undefined
        uint32 ext = readCsr32(REG_CSR_EXT_CTRL) & 0xCFFF;
        writeCsr32(REG_CSR_EXT_CTRL, ext | 1);

        Delay::ms(10);

        // We must poll the read-version of the suspend bit
        for(int i=0; i < 10000; i++)
        {
            uint32 ext = readCsr32(REG_CSR_EXT_CTRL) & 0xCFFF;
            if(ext & 1)
            {
                out << "Chip suspended successfully" << eol;
                break;
            }
            Delay::ms(1);
        }

        // Set promiscious mode (bit 15)
        uint32 mode = readCsr32(REG_CSR_MODE) & 0xFFFF;
        writeCsr32(REG_CSR_MODE, mode | (1 << 15));

        // Clear suspend bit to resume operation
        writeCsr32(REG_CSR_EXT_CTRL, ext);

        // After suspend, start and enable controller interrupts
        writeCsr32(REG_CSR_STATUS, 1<<CSR_STATUS_START | 1<<CSR_STATUS_IENA);
    }

    return E_OK;
}

/** @see NetworkInterface::write */
error_t PCNet32Driver::write(Buffer& buf)
{
    error_t res = E_OK;

    #if NOISE > 0
    out << "[NIC] Sending " << buf.size() << " bytes on descriptor " << (int)txTail << eol;
    #endif

    // Copy packet to tx desc buffer
    TxDescriptor* tx = (TxDescriptor*) txRing;

    if(tx[txTail].isAquired())
    {
        kassert(!tx[txTail].isErrors());

        // Send the frame; this copies the buffer into the descriptor buffer
        // and releases the descriptor.
        tx[txTail].transmit(buf, this);

        // Next descriptor
        txTail = (txTail + 1) % NUM_TXDESC;

        // Ask the device to check the tx descriptor immediately. Make sure the
        // reading and writing of the status register is atomic.
        AutoSpin lock(this);
        writeCsr32(REG_CSR_STATUS, readCsr32(REG_CSR_STATUS) | 1 << CSR_STATUS_TX_DEMAND);
    }
    else
    {
        out << "[NIC] NO TX DESCRIPTOR AVAILABLE" << eol;
        res = E_NO_RESOURCE;
    }

    #if NOISE > 0
    out << "STATUS: " << H(readCsr32(REG_CSR_STATUS)) << eol;
    #endif

    return res;
}

/** @see NetworkInterface::read */
error_t PCNet32Driver::read(Buffer& buf)
{
    NO_IMPL();

    return E_OK;
}

/**
 * Called by the Sender thread whenever packets are ready
 * to be transmitted in the output queue
 */
void PCNet32Driver::onSendData()
{
}

/**
 * Called by the Receiver thread whenever there are packets in
 * the rx buffer.
 */
void PCNet32Driver::onPacketsAvailable()
{
    #if NOISE > 0
    out << "[NIC] Rx interrupt" << eol;
    #endif

    // TODO: get descriptor status from dword2/3

    RxDescriptor* rx = (RxDescriptor*) rxRing;

    // Empty ready descriptors and transfer ownership back to the chip
    for(int i=0; i < NUM_RXDESC; i++)
    {
        if(!rx[rxDescIdx].isAquired())
            break;

        if(!rx[rxDescIdx].isErrors())
        {
            paddr physbuf = rx[rxDescIdx].getBuffer();
            int buflen = rx[rxDescIdx].getMessageLen();

            // Start moving up the stack.
            Buffer* frame = new Buffer(buflen);
            frame->copyFrom(toVirtual(physbuf), buflen);

            EthernetMgr::getInstance(this)->processFrame(frame);

            // Dump packet (irq disable due to 'out' not beeing locked)
            #if NOISE > 0
            flag_t flags;
            CPU::localIrqOff(&flags);

            out << "Packet in descriptor #" << rxDescIdx << " (" << buflen << " bytes): " << eol;
            Hexdump::dump((uint8*)toVirtual(physbuf), buflen);

            CPU::localIrqRestore(flags);
            #endif
        }
        else
        {
            uint32 dw2 = rx[rxDescIdx].getDword2();
            uint32 dw3 = rx[rxDescIdx].getDword3();

            // Note: handle errors / clear any error flags in RMD1/2
            out << "Errors in rx thread (dw2="<< H(dw2) << ", dw3="<< H(dw3) <<")" << eol;
        }

        // Hand the descriptor back to the chip
        rx[rxDescIdx].release();
        rxDescIdx = (rxDescIdx + 1) % NUM_RXDESC;
    }

    #if NOISE > 0
    out << "[NIC] Rx interrupt DONE" << eol;
    #endif
}


/**
 * NIC interrupt handler
 */
int PCNet32Driver::onInterrupt(ContextState* ctx)
{
    int res = IRQRES_NORMAL;

    uint64 start = Delay::getCPUCycles();

    // Get status from CSR0 (status) and check that this IRQ is of interest
    uint32 status = readCsr32(REG_CSR_STATUS);
    if(!Bits::isSet(status, CSR_STATUS_INTR))
    {
        out << "[NIC] Not ours... shared IRQ..." << eol;
        return 0;
    }

    // Get more status
    uint32 testreg = readCsr32(REG_CSR_TEST_FEATURES);
    uint32 extreg = readCsr32(REG_CSR_EXT_CTRL);

    if(Bits::isSet(status, CSR_STATUS_IDON))
    {
        out << "[NIC] Initialization block completed" << eol;

        if(isPromiscuous)
        {
            // Set promiscious mode (bit 15)
            uint32 mode = 1 << 15;
            writeCsr32(REG_CSR_MODE, mode);
        }

        // Start chip (the status is written back at the end of the ISR)
        status |= 1 << CSR_STATUS_START;
    }

    // Handle errors
    if(Bits::isSet(status, CSR_STATUS_ERR))
    {
        //out << "[NIC] Error interrupt, status =  " << H(status) << ", ext=" << H(extreg) << eol;

        // Missed frame is non-fatal; just update the count.
        if(Bits::isSet(status, CSR_STATUS_MISS))
        {
            // The miss frame count (mfc) is in CSR112
            uint32 mfc = readCsr32(112);

            // Report every 100 missed frame
            if((mfc % 100) == 0)
                out << mfc << " missed frames in total (#" << getObjectId() << ")" << eol;
        }

        // Collision error; may indicate detached cable
        if(Bits::isSet(status, CSR_STATUS_CERR))
        {
            out << "Collision error (detached cable?)" << eol;
        }

        // BABL and MERR are considered FATAL errors
        if(Bits::isSet(status, CSR_STATUS_BABL) || Bits::isSet(status, CSR_STATUS_MERR))
        {
            // This most likely is due to misconfigured buffers
            HALT("FATAL NIC ERROR");
        }
    }

    // If a packet has arrived, notify the receiver thread
    if(Bits::isSet(status, CSR_STATUS_RINT))
    {
        res = IRQRES_RESCHEDULE;
        receiver.notify(PRI_HIGH);
    }

    // A packet was sent
    if(Bits::isSet(status, CSR_STATUS_TINT))
    {
        TxDescriptor* tx = (TxDescriptor*) txRing;

        int i = 0;
        for(; i < NUM_TXDESC; i++)
        {
            if(tx[txHead+i].isAquired())
            {
                // tmd1 errors?
                if(tx[txHead+i].getDword2() & (1<<30))
                {
                    out << "TX ERROR" << eol;
                }

                uint32 dword3 = tx[txHead+i].getDword3();

                // buffer error
                if(dword3 & (1<<31))
                {
                    out << "TX BUFFER ERROR" << eol;
                    dword3 &= ~(1 << 31);
                }

                if(dword3 & (1 << 30))
                {
                    out << "TX UNDERFLOW ERROR" << eol;
                    dword3 &= ~(1 << 30);
                }

                if(dword3 & (1 << 28))
                {
                    out << "TX LATE COLLISION ERROR" << eol;
                    dword3 &= ~(1 << 28);
                }

                if(dword3 & (1 << 27))
                {
                    out << "TX CARRIER LOST" << eol;
                    dword3 &= ~(1 << 27);
                }

                if(dword3 & (1 << 26))
                {
                    out << "TX RETRY ERROR (16 attempts failed)" << eol;
                    dword3 &= ~(1 << 26);
                }

                tx[txHead+i].setDword3(dword3);

                //out << "DRIVER AQUIRED" << eol;

                break;
            }
            else
            {
                out << "BUFFER NOT TRANSMITTED YET" << eol;

                // tmd1 errors?
                if(tx[txHead+i].getDword2() & (1<<30))
                {
                    out << "TX ERROR" << eol;
                }
            }
        }

        txHead = (txHead + i) % NUM_TXDESC;
    }

    //out << "[NIC] Status: " << H(status) << eol;

    // We write back all the bits, since we handle everything we want to handle
    // in the single interrupt.
    writeCsr32(REG_CSR_STATUS, status);
    writeCsr32(REG_CSR_TEST_FEATURES, testreg);
    writeCsr32(REG_CSR_EXT_CTRL, extreg);

    uint64 time = Delay::usSince(start);
    //out << "[NIC] ISR INDUCED IRQ LATENCY: " << time << " us" << eol;

    return res;
}

/* See declaration */
paddr PCNet32Driver::toPhysRXBuffer(void* rx)
{
    return ((vaddr)&rxRing[NUM_RXDESC] - physDelta) + (((((vaddr)rx) - (vaddr)rxRing)/sizeof(RxDescriptor)) * RX_BUF_LEN);
}

/* See declaration */
paddr PCNet32Driver::toPhysTXBuffer(void* tx)
{
    return ((vaddr)&txRing[NUM_TXDESC] - physDelta) + (((((vaddr)tx) - (vaddr)txRing)/sizeof(TxDescriptor)) * TX_BUF_LEN);
}

}// namespace
