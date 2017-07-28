/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * Defines the NIC interface
 */

#ifndef _APOX_NIC_H_
#define _APOX_NIC_H_

#include <kernel/adt/Buffer.h>
#include <kernel/drivers/Device.h>
#include <kernel/io/net/IP.h>
#include <kernel/io/net/Ethernet.h>
#include <kernel/adt/List.h>

/* Properties keys for NIC devices */

#define PK_NIC_FRAMES_MISS          "Missed Frames"
#define PK_NIC_FRAMES_OK            "Good frames"
#define PK_NIC_FRAMES_ERROR         "Error frames"

/**
 * Abstract baseclass for NIC drivers
 */
class NetworkInterface : public DeviceDriver
{
    public:

        /** C'tor */
        NetworkInterface();

        /** C'tor */
        NetworkInterface(IPAddress& ipAddress);

        /** Test the interface; default implementation does nothing */
        virtual error_t test(int arg)
        { return 0; }

        /** Read packet into supplied buffer; this blocks (TODO: timeout?) */
        virtual error_t read(Buffer& buf) = 0;

        /** Write frame from supplied buffer. */
        virtual error_t write(Buffer& buf) = 0;

        /** Get hardware address */
        virtual Buffer& getHardwareAddress() = 0;

        /** Gets the MTU of the interface */
        int getMTU();

        /** Get list of IP addresses assigned to this interface. Mutable. */
        List<IPAddress*>& getIPAddresses();

    protected:

        /** Called by the Receiver thread whenever there are packets in the rx buffer */
        virtual void onPacketsAvailable() = 0;

        /** Sets the MTU of the interface */
        void setMTU(int mtu);

    private:

        /** List of IP addresses assoicated with the interface */
        List<IPAddress*> ipAddresses;

        /** Maximum transfer unit */
        int mtu;
};

#endif // _APOX_NIC_H_
