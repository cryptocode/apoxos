/**
 * Apox Operating System
 * Copyright 2006, cryptocode
 */

#ifndef _APOX_DRIVERS_NIC_RTL8139_H_
#define _APOX_DRIVERS_NIC_RTL8139_H_

#include <kernel/drivers/bus/pci/DriverPCI.h>
#include <kernel/drivers/nic/NIC.h>
#include <kernel/io/net/Ethernet.h>
#include <kernel/io/Ports.h>
#include <kernel/threads/Thread.h>

/** Internal driver name */
#define DRIVERNAME_RTL8139          "RTL8139 Compatible Network Interface"

/** Number of tx descriptors */
#define NUM_TXDESC                  4

/**
 * RTL8139 A/B/C/D chipset driver
 */
class RTL8139DDriver : public NetworkInterface, public PortIO
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
            Receiver(RTL8139DDriver* driver)
            {
                this->driver = driver;
            }

            /** @see Threadable#run */
            virtual void run(void* arg);

        private:

            RTL8139DDriver* driver;
    };


    /* Driver interface */

    public:

        /** Debugging tool */
        virtual error_t test(int param);

        /**
         * Get driver instance.
         *
         * @param dev The device we're gonna drive
         */
        static RTL8139DDriver* getInstance(PCIDevice* dev);

        /** D'tor */
        virtual ~RTL8139DDriver();

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
            static Buffer b(mac, ETH_MAC_BYTES);
            return b;
        }

    protected:

        /** Prepares the chip for online operation. Does everything but setting up and enabling IRQ */
        error_t prepare();

        /** Reset the chip */
        error_t reset();

        /** Send a packet */
        error_t sendPacket(uint8* packet);


    private:

        /** Private c'tor */
        RTL8139DDriver();

        /**
         * Logical 0-based consumer index to RTL's idea of an index starting at 0xFFF0
         * The producer index OTOH is 0 based.
         */
        inline uint16 logical2reg(uint16 in)
        {
            // This wrap around
            return in-16;
        }

        /** From virtual to physical address of receive or transmit buffer */
        inline paddr toPhys(void* va)
        {
            return vaddr(va) - physDelta;
        }

        inline vaddr toVirtual(paddr pa)
        {
            return vaddr(pa) + physDelta;
        }

        vaddr physDelta;

        /** Tx descriptor buffers */
        uint8* txBuffer[NUM_TXDESC];

        /** Rx buffer; no descriptor ring design for rx on this weird chip */
        uint8* receiveBuffer;

        /** Rx buffer offset */
        uint16 rxOffset;

        /** NIC's mac address */
        uint8 mac[ETH_MAC_BYTES];
        Buffer macadr;

        /** For stats; won't wrap around anytime soon */
        uint64 bytesReceived;

        /** Next descriptor to use */
        int nextDesc;

        /** Number of descriptors currently in use; max four. */
        Atomic txCnt;

        /** Reciver thread */
        Receiver receiver;

};

#endif // _APOX_DRIVERS_NIC_RTL8139_H_
