/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * Implemented according to the ATA/ATAPI-6 specification.
 *
 * Specifically, the document entitled "T13/1532D Volume 1 Revision 4b" was used.
 *
 * \ingroup ATA
 * @{
 */

#ifndef _APOX_DRIVER_ATA_H_
#define _APOX_DRIVER_ATA_H_

#include <kernel/libc/std.h>
#include <kernel/drivers/ata/ATA.h>
#include <kernel/drivers/Device.h>
#include <kernel/drivers/bus/pci/DriverPCI.h>
#include <kernel/io/Channel.h>
#include <kernel/adt/List.h>

/**
 * Manages the ATA bus
 */
class ATABus : public BusDriver
{
    public:

        ATABus();

        /** Create singleton */
        static ATABus* getInstance();

        /** Initialize ATA subsystem */
        static ATABus* initialize();

    private:

        void probe();
        void enumPCIDevices(PCIBus* bus);
        void enumATADevices();

        List<PCIDevice*> ideControllers;
        List<ATABus*> devices;

        DeviceDriver* driver;
        Channel* channel;

        /** Singleton */
        static ATABus* instance;
};

#endif // _APOX_DRIVER_ATA_H_

/**@}*/
