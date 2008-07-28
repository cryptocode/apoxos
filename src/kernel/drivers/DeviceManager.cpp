/**
 * Apox Operating System
 * Copyright (c) 2005, 2006, cryptocode
 */

#include <assert.h>
#include <kernel/Kernel.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/io/Channel.h>

DeviceManager* DeviceManager::instance = null;
List<Device*>* DeviceManager::devices = null;

/** Add a new device to the repository */
void DeviceManager::addDevice(Device* dev)
{
    if(dev->type == DEVTYPE_VFS)
    {
        Node* devNode = Kernel::getNamespace()->find(L"/dev/fs");
        assert(devNode != null);

        devNode->addChild(dev);
    }
    else if(dev->type == DEVTYPE_NIC)
    {
        Node* devNode = Kernel::getNamespace()->find(L"/dev/nic");
        devNode->addChild(dev);
    }

    devices->add(dev);
}

/** Dumb device repos to stdout */
void DeviceManager::dump()
{
    if(devices->count())
    {
        foreach(devices)
        {
            Device* dev = devices->currentItem();
            out << dev->getName()
                << ", Internal id = "<< H((vaddr)dev) << ", device object Id = " << dev->getObjectId();

            if(dev->type == DEVTYPE_VFS)
            {
                out << " (Filesystem)";
            }

            out << eol;
        }
    }
    else
    {
        out << "No devices registered" << eol;
    }
}
