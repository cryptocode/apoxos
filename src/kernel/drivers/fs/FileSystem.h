/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#ifndef _APOX_DRIVERS_FS_
#define _APOX_DRIVERS_FS_

#include <kernel/drivers/Device.h>
#include <kernel/Errors.h>
#include <assert.h>

class Node;
class Filesystem;

/**
 * A virtual filesystem device; i.e. a device capabable of doing VFS operations.
 */
class VFSDevice : public Device
{
    public:

        /** C'tor */
        VFSDevice(Filesystem* fs, String& name)
        {
            this->fs = fs;
            this->name = name;
            this->type = DEVTYPE_VFS;
        }

    private:

        /** The FS driver for this device */
        Filesystem *fs;
};

/**
 * File system driver base class
 */
class Filesystem : public DeviceDriver
{
    public:

        /** C'tor */
        Filesystem()
        {
        }

        /**
         * Mounts the file system in the user namespace
         *
         * Must be implemented to handling mounting in each concrete file system.
         *
         * @param parent Mount point
         * @return error E_OK, E_NO_IMPL
         */
        virtual error_t mount(Node* parent) = 0;


        /**
         * Called when a partition size request has been made. Returns SYSERR_NO
         * if the request was granted (and the FS did the required changes), or
         * SYSERR_NO_IMPL if unable to satisfy the request (this is the default
         * FS behaviour.)
         */
        virtual error_t onPartitionSizeChange(uint64 newSize)
        {
            assert(false);
            return E_NO_IMPL;
        }

    private:


};

#endif // _APOX_DRIVERS_FS_
