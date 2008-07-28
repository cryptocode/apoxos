/**
 * The Apox Operating System
 * Copyright (c) 2005, cryptocode
 *
 * \defgroup Devices
 * @{
 */

#ifndef _APOX_DEVICE_MANAGER_H_
#define _APOX_DEVICE_MANAGER_H_

#include <kernel/libc/std.h>
#include <kernel/adt/List.h>
#include <kernel/drivers/Device.h>

/**
 * Device manager; central repository of devices and drivers.
 */
class DeviceManager
{
    public:

        /** Add a new device to the repository */
        static void addDevice(Device* dev);

        /** Finds a device by name. Returns NULL if not found */
        static Device* findByName(String const& name)
        {
            Device* res = null;

            foreach(devices)
            {
                if(devices->currentItem()->getName().compare(name) == 0)
                {
                    res = devices->currentItem();
                    break;
                }
            }

            return res;
        }

        /** Finds a device by object id. Returns NULL if not found */
        static Device* findByObjectId(oid_t oid)
        {
            Device* res = null;

            foreach(devices)
            {
                if(devices->currentItem()->getObjectId() == oid)
                {
                    res = devices->currentItem();
                    break;
                }
            }

            return res;
        }

        /** Dumb device repos to stdout */
        static void dump();

        /** Creates and/or returns singleton */
        static DeviceManager* initialize()
        {
            if(instance == null)
            {
                instance = new DeviceManager();
            }

            return instance;
        }

    protected:

        /** C'tor */
        DeviceManager()
        {
            devices = new List<Device*>;
        }

    private:

        static DeviceManager* instance;
        static List<Device*>* devices;
};

#endif // _APOX_DEVICE_MANAGER_H_

/**@}*/
