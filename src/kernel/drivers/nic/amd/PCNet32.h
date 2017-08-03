/*
 * Apox Operating System
 * Copyright 2006, cryptocode
 */

#ifndef _APOX_DRIVERS_NIC_PCNET32_H_
#define _APOX_DRIVERS_NIC_PCNET32_H_

#include <kernel/drivers/bus/pci/DriverPCI.h>
#include <kernel/drivers/nic/NIC.h>
#include <kernel/io/net/Ethernet.h>
#include <kernel/io/Ports.h>
#include <kernel/threads/Thread.h>

namespace PCNet32
{

/** Internal driver name */
#define DRIVERNAME_PCNET32          "AMD PCNet32 Compatible Network Interface"

/** Number of transmit descriptors in the tx ring */
#define NUM_TXDESC_LOG2             8
#define NUM_TXDESC                  (1 << NUM_TXDESC_LOG2)

/** Number of receive descriptors in the rx ring; 2^8 = 256 - enough to keep missed
 *  frames to a minimum. */
#define NUM_RXDESC_LOG2             8
#define NUM_RXDESC                  (1 << NUM_RXDESC_LOG2)

class RxDescriptor;
class TxDescriptor;

/**
 * RTL8139 A/B/C/D chipset driver
 */
class PCNet32Driver : public NetworkInterface, public PortIO
{
    /**
     * Packet reception thread; woken up whenever there is something of interest in
     * the rx buffer. All run() does is call back to onPacketAvailable in the Receiver
     * threads' context.
     */
    class Receiver : public Threadable, public Waitable
    {
        public:

            /** C'tor */
            Receiver(PCNet32Driver* driver)
            {
                this->driver = driver;
            }

            /** @see Threadable#run */
            virtual void run(void* arg);

        private:

            PCNet32Driver* driver;
    };

    /**
     * Packet transmission thread; woken up when there is something to send.
     */
    class Sender : public Threadable, public Waitable
    {
        public:

            /** C'tor */
            Sender(PCNet32Driver* driver)
            {
                this->driver = driver;
            }

            /** @see Threadable#run */
            virtual void run(void* arg);

        private:

            PCNet32Driver* driver;
    };

    /* Driver */

    public:

        /** Debugging tool */
        virtual error_t test(int param);

        /** Send a packet */
        error_t sendPacket(uint8* packet);

        /**
         * Get driver instance.
         *
         * @param dev The device we're gonna drive
         */
        static PCNet32Driver* getInstance(PCIDevice* dev);

        /** D'tor */
        virtual ~PCNet32Driver();

        /**
         * Called by the Receiver thread whenever there are packets
         * in the rx buffer
         */
        void onPacketsAvailable();

        /**
         * Called by the Sender thread whenever packets are ready
         * to be transmitted in the output queue
         */
        void onSendData();

    /* Implements DeviceDriver */
    public:

        /**
         * NIC interrupt handler
         */
        virtual int onInterrupt(ContextState* ctx);


        /** @see DeviceDriver::setStatus */
        virtual error_t setStatus(DeviceStatus status);

    /* Implements NetworkInterface */
    public:

        /** @see NetworkInterface::read */
        virtual error_t read(Buffer& buf);

        /** @see NetworkInterface::write */
        virtual error_t write(Buffer& buf);

        /** Get hardware address */
        virtual Buffer& getHardwareAddress()
        {
            //static Buffer b(mac, ETH_MAC_BYTES);
            return macadr;
        }

    /* Descriptor support methods */
    public:

        inline paddr toPhys(void* ptr)
        {
            return ((vaddr)ptr) - physDelta;
        }

        inline void* toVirtual(paddr phys)
        {
            return (void*)(phys + physDelta);
        }

        paddr toPhysRXBuffer(void* ptr);
        paddr toPhysTXBuffer(void* ptr);

    protected:

        /** Prepares the chip for online operation. Does everything but setting up and enabling IRQ */
        error_t prepare();

        /** Reset the chip */
        error_t reset();

        void writeBcr32(int reg, uint32 data);
        uint32 readBcr32(int reg);
        void writeCsr32(int reg, uint32 data);
        uint32 readCsr32(int reg);

    private:

        /** Private c'tor */
        PCNet32Driver();


    private:

        /** Promiscuous mode */
        bool isPromiscuous;

        /**
         * The difference between the virtual and phys address of the
         * ring buffer arena
         */
        vaddr_int physDelta;

        /** Rx descriptor array */
        RxDescriptor* rxRing;

        /** Tx descriptor array */
        TxDescriptor* txRing;

        /** Head of tx ring */
        int txHead;

        /** Current tail of tx ring */
        int txTail;

        /** Current receive descriptor */
        int rxDescIdx;

        /** NIC's mac address */
        uint8 mac[ETH_MAC_BYTES];
        Buffer macadr;

        /** For stats; won't wrap around anytime soon */
        uint64 bytesReceived;

        /** Reciver thread */
        Receiver receiver;

        /** Sender thread */
        Sender sender;
};

} // namespace

#endif // _APOX_DRIVERS_NIC_PCNET32_H_
