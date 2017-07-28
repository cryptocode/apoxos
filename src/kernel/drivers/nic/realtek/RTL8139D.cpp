/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

/**
 * Primary reference is the RTL8139D data sheet.
 *
 * This implements a driver for the RTL8139 chipset from Realtek that several
 * NIC vendors use.
 *
 * Overview
 *
 * 1. Packet Reception
 *    The driver allocates a 32-bit aligned, physically contigious memory area
 *    of (8192 << x) bytes where x is 0..3. So up to 64K (+ we're allocating
 *    some wrap padding).
 *
 *    The chip moves data to this buffer by means of PCI DMA.
 *
 *    The chip provides us a consumer register (where we are reading from)
 *    called CAPR and a producer register (where the chip is writing) called CBR.
 *
 *    Our main task is to read anything between the consumer and producer
 *    indices and advance the consumer index (and handle ring buffer wrapping,
 *    etc). Note that the consumer index is a bit weird since it starts of at
 *    0xFFF0.
 *
 *    As packets arrive, the irq handler wakes up the Receiver thread which in
 *    turns reads all unread data from the rx ring buffer and subsequently
 *    starts moving the packets up the proper network stack path.
 *
 * 2. Packet transmission
 *    The chip has 4 descriptor registers for transmission where each frame
 *    transmitted is from a physically contiguous, 32-bit aligned buffer. The
 *    atomic txCnt keeps track of how many buffers are in use. The descriptors
 *    *must* be used in a round-robin fashion or the chip will complain.
 *
 * 3. Chip bugs and poorly or undocumented (mis)features
 *
 *    a) The "B" version is claimed to be unstable when media autoselect is on.
 *    b) The 64K buffer length setting is claimed to occasionally lock up the
 *       chip.
 *    c) If packet length on reception is 0xfff0, it actually means the
 *       chip<->memory DMA operation is still in flight.
 *    d) Must activate Tx/Rx before configuring Tx/Rx
 *
 * 4. Driver bugs and lackings
 *
 *    a) No configuration abilities
 *    b) Error checking and handling needs improvement
 *    c) Power mgmt
 *    d) Promiscuous mode-switch upport
 */

#include <kernel/drivers/nic/realtek/RTL8139D.h>
#include <kernel/io/Channel.h>
#include <kernel/io/net/ARP.h>
#include <kernel/io/net/IP.h>
#include <kernel/init/irq.h>
#include <kernel/threads/Thread.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/util/Hexdump.h>
#include <kernel/New.h>
#include <kernel/vmm/vmm.h>
#include <kernel/drivers/bus/pci/PCIDatabase.h>
#include <arch/drivers/bus/pci/PCIConfig.h>
#include <arch/Delay.h>

namespace
{

/** Nominal buffer size; this is the boundary at which we wrap */
#define RX_BUFFER_SIZE_NOMINAL(config)  (8192 << config)

/** Computes the required buffer size, given the config (0=8k, 1=16K, 2=32K, 3=64K) */
#define RX_BUFFER_SIZE_ACTUAL(config)   ((RX_BUFFER_SIZE_NOMINAL(config)) + 16 + 1536)

#define	RX_BUFFER_SIZE		            RX_BUFFER_SIZE_NOMINAL(0x3)
#define RX_MAX_PACKET_LEN	            1600
#define RX_MIN_PACKET_LEN	            64
#define	RX_READ_PTR_MASK	            0x3FFC

/** All RX related isr's */
#define RX_ISR_ALL_MASK                 (0x40 | 0x10 | 0x01)

/*
 * REGISTERS
 *
 * These are all byte offsets into the RTL 8139D register space.
 * Do the proper division for 2 and 4 byte access.
 */

/** Read 1,2,4 bytes, write 4 byte access; MAC address */
#define REG_ID                          0x00

/** Read 1,2,4 bytes, write 4 byte access */
#define REG_MULTICAST                   0x08

/* Transmission status registers.
 * The driver exploits the fact the reg offsets are in arithmetic progression. */
#define REG_TX_STATUS0                  0x10
#define REG_TX_STATUS1                  0x14
#define REG_TX_STATUS2                  0x18
#define REG_TX_STATUS3                  0x1C

/* Transmission descriptors; start address of physically contiguous packet buffer.
 * The driver exploits the fact the reg offsets are in arithmetic progression. */
#define REG_TX_DESC0                    0x20
#define REG_TX_DESC1                    0x24
#define REG_TX_DESC2                    0x28
#define REG_TX_DESC3                    0x2C

/** Lower 13 bit of status registers contains the byte count */
#define REG_TX_STATUS_BYTECOUNT_MASK    0x00001FFF

/** Length of each TX descriptor's frame buffer */
#define TX_FRAMEBUF_LEN                 2048

/** Transmit status flags; the symbol names corresponds to the data sheet names */
enum TXStatusBits
{
    /** Carrier Sense Lost */
    TXS_CRS = 31,

    /** Transmit Abort */
    TXS_TABT = 30,

    /** Out of Window Collision. */
    TXS_OWC = 29,

    /** CD Heart Beat; cleared in 100Mbps mode. */
    TXS_CDH = 28,

    /** When 1, packet was completed successfully with no underruns */
    TXS_TOK = 15,

    /**
     * Tx FIFO underrun. If the early tx threshold is too low and the bus is
     * really busy, this condition may occur (i.e. the host is unable to feed
     * the FIFO fast enough.) Usual counter measure is to bump up the tx
     * threshold with the tradeoff that data ships to line with a bit higher
     * latency. */
    TXS_TUN = 14,

    /**
     * The chip sets this bit to 1 when the tx DMA op of the descriptor is
     * completed. The driver sets it to 0 when the start address and byte count
     * has been written to start the DMA operation.
     */
    TXS_OWN = 13,
};

/** Command register; 1 byte */
#define REG_CMD                 0x37

/**
 * Current Address of Packet Read, CAPR (aka consumer index); 1 byte.
 * This register is updated by Apox after packet reception.
 */
#define REG_CONSUMER_IDX        0x38

/** Current buffer address, CBA (aka producer index) */
#define REG_PRODUCER_IDX        0x3A

/** Interrupt Mask Reg; 2 bytes */
#define REG_IMR                 0x3C

/** Interrupt Status Reg; 2 bytes */
#define REG_ISR                 0x3E

/** Physical receive buffer; 4 bytes */
#define REG_RX_STARTADDR        0x30

/** Receive Config Reg; 4 bytes */
#define REG_RX_CONFIG           0x44

/** Missed Packet Counter; 24-bit counter indicating number of discared packets
 * due to Rx FIFO overflow */
#define REG_MPC                 0x4C

/** Timer interrupt register; 4 bytes */
#define REG_TIMERINT		    0x54

/** Media status; 1 byte */
#define REG_MEDIA_STATUS        0x58

/** Multiple Interrupt Select Register. We use this to turn off early rx irq's */
#define REG_MULTI_IRQ           0x5C

/** CS Configuration Register; 2 bytes */
#define REG_CSCR                0x74
#define BIT_CSCR_CON_STATUS     3

/** PCI command reg; read as 16-bit */
#define PCI_REG_CMD             0x04

/** PCI byte offset of I/O BAR; read as 32-bit */
#define PCI_REG_IOBAR           0x10

/** 93C46 command register */
#define REG_CFG_9346		    0x50
#define CFG_9346_WRITEENABLE    0xC0
#define CFG_9346_WRITEDISABLE   0x00

/** Configuration register 0; 1 byte */
#define REG_CFG_0	            0x51

/** Configuration register 1; 1 byte */
#define REG_CFG_1		        0x52

/** Configuration register 3; 1 byte */
#define REG_CFG_3		        0x59

/** Configuration register 4; 1 byte */
#define REG_CFG_4		        0x5A


/** Signals reset to rx/tx threads */
#define SIG_RESET               1

/** Signals thread exit to rx/tx threads */
#define SIG_THREAD_EXIT         2

}// anon namespace

/**
 * C'tor
 */
RTL8139DDriver::RTL8139DDriver() : macadr(mac, ETH_MAC_BYTES), receiver(this)
{
    rxOffset = 0;

    /*
     * Arena layout:
     *
     * N TRANSMIT BUFFERS
     * 1 RECEIVE BUFFER
     *
     * No requirements beyound 32 bit alignment.
     *
     * Receive buffer: only works cuz' we're ID mapped.
     * We add 8 bytes since we're going to set the start buffer 8 bytes into this
     * buffer for obscure reasons (the chip may prepend data)
     */
    size_t arenasize = NUM_TXDESC*TX_FRAMEBUF_LEN + RX_BUFFER_SIZE_ACTUAL(0x3) + 8;

    paddr pa; vaddr va;
    size_t arenapages = Math::divup(arenasize, VM_PAGE_SIZE);
    assert(VMM::allocMappedRegion(va, pa, arenapages) == E_OK);
    memset((void*)va, 0, arenasize);

    vaddr arena = va;
    physDelta = va - pa;

    for(int i=0; i < NUM_TXDESC; i++)
        txBuffer[i] = reinterpret_cast<uint8*>(va + (i*TX_FRAMEBUF_LEN));

    receiveBuffer = reinterpret_cast<uint8*>(va + NUM_TXDESC*TX_FRAMEBUF_LEN + 8);

    bytesReceived = 0;
    nextDesc = 0;

    setMTU(1500);

    // TODO: just testing
    getIPAddresses().add(new IPAddress("10.0.0.99"));
}

/** D'tor */
RTL8139DDriver::~RTL8139DDriver()
{
    // NOTE: Add arena deallocation
}

/** @see Threadable#run */
void RTL8139DDriver::Receiver::run(void* arg)
{
    for(;;)
    {
        // Wait 'till there's stuff to do
        wait();

        // The chip was stopped for restart and the notify count might have been > 0
        // so we have to reset the wait count.
        if(getSignal() == SIG_RESET)
        {
            Thread::sleep(1000);

            setSignal(0);

            driver->setStatus(DEVSTATUS_RUNNING);
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

/**
 * Get (and if necessary create) a driver instance for the device
 */
RTL8139DDriver* RTL8139DDriver::getInstance(PCIDevice* dev)
{
    if(dev->getDriver() == null)
    {
        RTL8139DDriver* driver = new RTL8139DDriver();
        driver->setDevice(dev);
        dev->setDriver(driver);

        // Register ourselves; we'll automatically be put in the NIc registry.
        String name(DRIVERNAME_RTL8139);
        dev->setName(name);

        DeviceManager::addDevice(dev);
        out << "Created RTL8139DDriver, IRQ " << dev->getIrqLine() << ", Dev: "
            << dev->deviceNum << ", FnID: " << dev->functionNum << eol;

        if(driver->setStatus(DEVSTATUS_RUNNING) == E_OK)
        {
            // Create receiver thread; it immediatly puts itself in a wait condition and
            // is woken up via notify() as packets arrive.
            Thread::create(&driver->receiver);
        }

    }

    return (RTL8139DDriver*)dev->getDriver();
}

/**
 * Change device status
 *
 * @see DeviceDriver::setStatus
 */
error_t RTL8139DDriver::setStatus(DeviceStatus status)
{
    error_t res = E_OK;

    if(status == DEVSTATUS_STOPPED || status == DEVSTATUS_SUSPENDED)
    {
        // Disable DMA RX and TX
        out8(REG_CMD, 0);

        // Mask all interrupts and unregister the irq handler
        out16(REG_IMR, 0);
        uninstallIRQHandler(getDevice()->getIrqLine(), this);

        // TODO: dealloc tx buffers, turn off nic clock to save power, etc

        out << "[NIC] Device stopped" << eol;
    }
    else if(status == DEVSTATUS_RUNNING)
    {
        // Put the chip and the driver into a sane state
        res = prepare();

        if(res == E_OK)
        {
            // Register irq handler
            installIRQHandler(getDevice()->getIrqLine(), this);

            // Accept all interrupts
            out16(REG_IMR, 0xFFFF);

            out << "[NIC] Started ";

            // Get speed; 0=100Mbps, 1=10Mbps
            if(Bits::isSet(in8(REG_MEDIA_STATUS), 3))
                out << "in 10 Mbps mode" << eol;
            else
                out << "in 100 Mbps mode" << eol;
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
error_t RTL8139DDriver::reset()
{
    out << "[NIC] Resetting" << eol;
    out8(REG_CMD, 1 << 4);
    uint8 cmdReg;

    // Give the chip a breather before checking for reset completion
    Delay::ms(10);

    // Check flag
    int retryCount = 1000;
    do
    {
        // This will matter when doing i/o mapped communications with the
        // chip(?) rather than the current port i/o approach
        WMB();

        // The reset is complete when bit 4 is cleared
        cmdReg = in8(REG_CMD);
        Delay::us(10);

    }while(Bits::isSet(cmdReg, 4) && retryCount-- > 0);

    out << "[NIC] Reset completed with retrycount = " << retryCount << eol;

    // Wait a bit before touching the card after a reset
    Delay::ms(5);

    return retryCount > 0 ? E_OK : E_TIMEOUT;
}


/**
 * Prepares the chip for online operation.
 * Does everything but setting up and enabling IRQ
 */
error_t RTL8139DDriver::prepare()
{
    error_t res = E_OK;

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

    out << "PCI IO base: " << H(iobase) << eol;

    // Reset the chip
    res = reset();
    // FIXME: this sux; how to handle error propagation ?!?!?!
    if(res != E_OK)
        return res;

    // Get Bus Master Enable status; set if required
    uint32 cmdPciReg;
    PCIConfig::read(dev->getBus()->getBusNumber(), dev->deviceNum,
                    dev->functionNum, PCI_REG_CMD/2 , 2, &cmdPciReg);

    out << "[NIC] PCI cmd reg: " << H(cmdPciReg) << eol;
    if(!Bits::isSet(cmdPciReg,2))
    {
        cmdPciReg |= (1<<2);

        out << "[NIC] PCI Bus Master enable beeing set manually now" << eol;
        PCIConfig::write(dev->getBus()->getBusNumber(), dev->deviceNum,
                         dev->functionNum, PCI_REG_CMD/2 , 2, cmdPciReg);
    }

    // Program tx start addresses (must be done each time since we've reset the
    // chip)
    for(int i=0; i < NUM_TXDESC; i++)
    {
        // TODO: toPhys
        out32(REG_TX_DESC0+(i*4), (uint32)toPhys(txBuffer[i]));
    }

    // Get MAC address
    out << "MAC address: ";
    for(int i=0; i < ETH_MAC_BYTES; i++)
    {
        mac[i] = in8(i);

        if(i > 0)
            out << ":";

        out << H(in8(i), false, 2);
    }
    out << eol;

    // Enable RX and TX *before* configuring
    out8(REG_CMD, 0xC);

    // Turn off early rx interrupts to prevent FIFO-in-progress packet data
    // Upper four bits are reserved, so we read and preserve them.
    // TODO: should we really do this?
    out16(REG_MULTI_IRQ, in16(REG_MULTI_IRQ) & 0xF000);

    // Set receive buffer start address.
    out32(REG_RX_STARTADDR, (uint32)toPhys(receiveBuffer));

    // RX configuration. We set the rx buffer size to 64K+16 bytes (11b)
    uint32 rxConfig=0;

    // Unlimited DMA burst size per Rx DMA burst
    // rxConfig = Bits::setRange(rxConfig, 8,10, 0x7);

    // DMA burst: 256 bytes
    rxConfig = Bits::setRange(rxConfig, 8,10, 0x4);

    // 8K+16 ring
    //rxConfig = Bits::setRange(rxConfig, 11,12, 0x0);
    // 64K+16 bytes (has HW problems?)
    rxConfig = Bits::setRange(rxConfig, 11,12, 0x3);
    out << "Nominal buffer size : " << RX_BUFFER_SIZE_NOMINAL(0x3) << eol;
    out << "Actual buffer size  : " << RX_BUFFER_SIZE_ACTUAL(0x3) << eol;

    // No Rx FIFO threshold =>  transfer FIFO to mem in entire frames
    //rxConfig = Bits::setRange(rxConfig, 13,15, 0x7);

    // Rx FIFO Threshold: 256 bytes
    rxConfig = Bits::setRange(rxConfig, 13,15, 0x4);

    // Allow all types of packets (categories: error, runt, broadcast,
    // multicast, physical match, all)
    //rxConfig |= 0x3F;

    // Everything but runt and error packages
    rxConfig |= 0xF;

    // Start on descriptor 0
    nextDesc = 0;
    txCnt = 0;
    // Wrap (invalid in 64K mode)
    //rxConfig |= (1 << 7);

    // Effectuate config
    out32(REG_RX_CONFIG, rxConfig);
    out << "[NIC] RX Config: " << H(rxConfig) << eol;

/*
    // Configure the 9346
    out8(REG_CFG_9346, CFG_9346_WRITEENABLE);

    // Reset config 0
    out8(REG_CFG_0, 0);

    // Turn off lan-wake and set the driver-loaded bit
    out8(REG_CFG_1, (in8(REG_CFG_1) & ~0x30) | 0x20);

    // Turn on FIFO auto-clear
    out8(REG_CFG_4, in8(REG_CFG_4) | 0x80);

    // Done configuring the 9346
    out8(REG_CFG_9346, CFG_9346_WRITEDISABLE);
*/

    // Initialize multicast regs
    for(int i = 0; i < 8; i++)
    {
        out8(REG_MULTICAST+i, 0xFF);
    }

    // Reset rx buffer offset
    rxOffset = 0;

    // Reset Missed Packet Counter
    out32(REG_MPC, 0);

    return res;
}

/** Send a packet */
error_t RTL8139DDriver::sendPacket(uint8* _packet)
{
    return E_OK;
}

/**
 * Test interface
 */
error_t RTL8139DDriver::test(int param)
{
    if(param == 0)
    {
        out << "   tnic1: Dump first 1K of the RX buffer" << eol;
        out << "   tnic2: Send ARP packet to broadcast fixed IP " << eol;
        out << "   tnic3: Restart NIC " << eol;
        out << "   tnic4: Suspend NIC " << eol;
        out << eol;
    }
    if(param == 1)
    {
        out << "[NIC] RECEIVEBUFFER:\n-------------------------------------" << eol;

        // Dump the first 1K
        Hexdump::dump(receiveBuffer, 1024);

        out << "----------------------------------------------" << eol;
    }
    else if(param == 2)
    {
        if(getStatus() == DEVSTATUS_RUNNING)
        {
            // Send N packets
            for(int i=0; i< 10; i++)
            {
                sendPacket(null);
            }
        }
        else
        {
            out << "The NIC is not running; try tnic3 first" << eol;
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

    return E_OK;
}

/** @see NetworkInterface::write */
error_t RTL8139DDriver::write(Buffer& buf)
{
    error_t res = E_OK;

    // retry for 100 ms in 5 ms increments
    for(int i=0; i < 20; i++)
    {
        if(txCnt.toLong() < 4)
        {
            ++txCnt;
            break;
        }
        else
        {
            out << "RETRYING TX BUFFER, cnt = " << (int)txCnt << eol;
            Thread::sleep(5);
        }
    }

    if((int)txCnt == 4)
    {
        out << "NO FREE TX BUFFER" << eol;
        res = E_NO_RESOURCE;
        return res;
    }

    buf.copyTo(txBuffer[nextDesc]);

    // This RTL chip doesn't autopad, so we'll have to do it. The TX buffer
    // is more than large enough.
    size_t packetSize = buf.size();
    if(packetSize < ETH_MIN_FRAMESIZE)
    {
        // We just adjust the size and don't care about zero-filling
        // the padded area.
        packetSize = ETH_MIN_FRAMESIZE;
    }

    // Set buffer (we can't free it with the current design)
    // The descriptors registers are consequtive, 4 bytes apart
    out32(REG_TX_DESC0+(nextDesc*4), (uint32)toPhys(txBuffer[nextDesc]));

    // Sets length and clears OWN to effectuate
    out32(REG_TX_STATUS0+(nextDesc*4), packetSize | 0x80000);

    // GET NEXT DESCRIPTOR
    nextDesc = (nextDesc+1) % 4;

    return res;
}

/** @see NetworkInterface::read */
error_t RTL8139DDriver::read(Buffer& buf)
{
    // Precond
    if(buf.size() < 1500)
    {
        return Thread::setLastError(E_INVALID);
    }

    uint8* ptr = buf;

    return E_OK;
}

/**
 * Called by the Sender thread whenever packets are ready
 * to be transmitted in the output queue
 */
void RTL8139DDriver::onSendData()
{
    NO_IMPL();
}

/**
 * Called by the Receiver thread whenever there are packets in the rx buffer
 */
void RTL8139DDriver::onPacketsAvailable()
{
    // While buffer is not empty
    do
    {
        // END
        uint16 producerIndex = in16(REG_PRODUCER_IDX);

        // START
        uint16 consumerIndex = (in16(REG_CONSUMER_IDX) + 16);

        if((consumerIndex == producerIndex) && (in8(REG_CMD) & 1) == 1)
        {
            out << ", Buff Emtpy; nothing to do " << eol;
            return;
        }

        // Extract packet length (including CRC)
        uint32 ringOffset = consumerIndex % RX_BUFFER_SIZE;

        // Header is 16-bit packet length followed by 16-bit rx status
        uint32 rxHeader = Convert::le2host((uint32)*(volatile uint32*) (receiveBuffer + ringOffset));
        uint16 rxStatus = rxHeader & 0xFFFF;
        uint16 rxLength = rxHeader >> 16;
        uint16 packetLen = rxLength - 4 /* CRC length is included in len */;

        // This indicates an unfinished entry, i.e. the RTL chip -> memory
        // process is still in flight.
        if(packetLen == 0xFFF0)
        {
            out << " ----  UNFINISHED PACKET ---- " << eol;
            return;
        }

        // Error in packet
        if((rxStatus & 1 ) == 0)
        {
            out << "[NIC] Errornous packet, status: " << H(rxStatus) << eol;

            // The receiver thread restarts the driver when it seens signal#1.
            // TODO: a bit brutal to reset the nic on each errornous packet? Seems to
            // happened a lot when using nmap, etc.
            receiver.setSignal(1);
            setStatus(DEVSTATUS_STOPPED);

            return;
        }
        // TODO:

        bytesReceived += packetLen;

        //out << ", Pck LE: " << packetLen;

        // Frame status, called "Receive Status Register in Rx Packet Header" in the data sheet (!)
        // TODO: A/B spec says status is *in front* of header
        //out << ", Stat=" << H(rxStatus);

        if(Bits::isSet(rxStatus, 13))
        {
            //out << " BROADCAST ";
        }
        else if(Bits::isSet(rxStatus, 14))
        {
            //out << " PHYS MATCHED";
        }
        else if(Bits::isSet(rxStatus, 15))
        {
            //out << " MULTICAST ";
        }

        // Packet data (+4 due to 32-bit NIC header)
        uint32 pckOffset = ringOffset+4;

        Buffer* frame = new Buffer(packetLen);
        frame->copyFrom((const void*)&receiveBuffer[pckOffset], packetLen);

        EthernetMgr::getInstance(this)->processFrame(frame);

        // Compute and set new consumer index
        rxOffset = ((rxOffset + rxLength + 4 + 3) & ~3);
        out16(REG_CONSUMER_IDX, logical2reg(rxOffset));

        //out << "\nNEW rxOffset/ringOffset: " << H(rxOffset) << "/" << H(ringOffset) << " ";
        //out << "\n New logical CIDX: " << H(reg2logical(in16(REG_CONSUMER_IDX))) << eol;

        producerIndex = in16(REG_PRODUCER_IDX);
        //out << "  -- PIDX " << producerIndex << eol;
        //out << "  -- CIDX " << in16(REG_CONSUMER_IDX) << eol;

        if(producerIndex == (rxOffset+16))
        {
            if((in8(REG_CMD) & 1) == 0)
                out << "BUFFER FULL... PACKETS MAY GET DROPPED" << eol;
            else
                out << "BUFFER EMPTY" << eol;

            // out << "\n PIDX is now: " << H(producerIndex) << eol;
            // out << "\n   We're behind (producerIndex != rxOffset) => (" << H(producerIndex) << ", " << H(rxOffset) << ")";
        }

        bool more = producerIndex < rxOffset || (rxOffset+16 < producerIndex);

        if(!more)
            break;

    } while(true);
}


/**
 * NIC interrupt handler
 */
int RTL8139DDriver::onInterrupt(ContextState* ctx)
{
    // Get interrupt status
    uint16 isr = in16(REG_ISR);

    // Shared irq? (note that we handle *all* interrupts on this nic, thus no mask checking here)
    if(isr == 0)
    {
        return 0;
    }

    if(isr == 0xFFFF)
    {
        out << "[NIC] The NIC appears to be detached\n";

        // Ack
        out16(REG_ISR, isr);
        return 0;
    }

    if(isr & 0x8000)
    {
        out << "[NIC] PCI SERR";

        // We need a full chip restart here
        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);

        return 0;
    }

    if(isr & 0x4000)
    {
        out << "[NIC] Timeout";
    }

    if(isr & 0x2000)
    {
        out << "[NIC] Cable length change";
    }

    if(isr & 0x40)
    {
        out << "[NIC] Rx FIFO Overflow";

        // Ack all RX isr's
        out16(REG_ISR, RX_ISR_ALL_MASK);

        return 0;
    }

    if(isr & 0x20)
    {
        // This can theoretically be a Packet Underrun or a Link change
        // notification, but treat it as the latter only.
        uint8 mediaStatus = in8(REG_MEDIA_STATUS);
        if(Bits::isSet(mediaStatus, 2))
        {
            out << "[NIC] Cable unplugged" << eol;
        }
        else
        {
            out << "[NIC] Cable plugged in" << eol;
        }
    }

    if(isr & 0x10)
    {
        out << "[NIC] Rx Buffer overflow after " << bytesReceived << " bytes " << eol;

        // TODO: update CAPR? spec is unclear.

        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);

        // Ack all RX isr's
        out16(REG_ISR, RX_ISR_ALL_MASK);

        return 0;
    }

    // Tx Error is only assert upon excessive collisions; should maybe instruct
    // the sender thread to sleep for a while?
    if(isr & 0x8)
    {
        out << "[NIC] Tx Error" << eol;

        // Must reset (maybe just resetting the transmitter [how?] is sufficient)
        setStatus(DEVSTATUS_STOPPED);
        setStatus(DEVSTATUS_RUNNING);

        out16(REG_ISR, 0x04);
        return 0;
    }

    if(isr & 0x4)
    {
        //out << "[NIC] Tx OK received, cnt = " << (int)txCnt << eol;

        // Read status
        for(int i=0; i < 4; i++)
        {
            uint32 st = in32(REG_TX_STATUS0+i*4);

            // Don't touch descriptor we don't own
            if(!Bits::isSet(st, TXS_OWN))
            {
                int collisions = (st >> 24) & 0xF;
                if(collisions > 0)
                    out << "COLLISIONS DETECTED: " << collisions << eol;

                // Seems like TOK can be combined with error flags
                if(Bits::isSet(st, TXS_TUN)  || Bits::isSet(st, TXS_OWC) ||
                   Bits::isSet(st, TXS_TABT) || Bits::isSet(st, TXS_CRS))
                {
                    out << "TX ERROR: Status" << i << " =  " << H(st) << eol;
                }
                else if(Bits::isSet(st, TXS_TUN))
                {
                    out << "Tx UNDERRUN... increasing threshold" << eol;
                }
                else if(!Bits::isSet(st, TXS_TOK))
                {
                    out << "TX ERROR (not TUN, OWC, TABT or CRS): Status"
                        << i << " =  " << H(st) << eol;
                }
            }
        }

        // At least one descriptor is free
        --txCnt;

        // Ack and return
        out16(REG_ISR, 0x04);
        return 0;
    }

    // CRC or alignment error; ignore
    if(isr & 0x2)
    {
        out << "[NIC] Rec Error" << eol;
        out16(REG_ISR, 0x2);
        return 0;
    }

    if(isr & 0x1)
    {
        // Wake up the receiver thread, if not already up'n'running.
        receiver.notify();

        // Ack all RX isr's
        out16(REG_ISR, RX_ISR_ALL_MASK);

        return IRQRES_RESCHEDULE;
    }

    // Ack
    out16(REG_ISR, isr);

    return 0;
}
