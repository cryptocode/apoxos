/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#ifndef _APOX_DRIVERS_FS_GENERIC_PARTITIONS_H_
#define _APOX_DRIVERS_FS_GENERIC_PARTITIONS_H_

#include <kernel/drivers/Device.h>
#include <kernel/io/Channel.h>
#include <kernel/Errors.h>

/**
 * The partition manager parses the partition table, given a channel (usually
 * interfacing with an ATA device or a floppy device)
 *
 * Each Partion is handed out as a Filesystem instance. The proper Filesystem
 * subclass instance is given a Channel implemented by PartitionManager so that
 * each Filesystem believes it has the disk all on it's own.
 *
 * The Channel (actually a PartitionChannel) has the proper partition offset and
 * capacity.
 */
class PartitionManager : public Object
{
    public:

        /** Creates a partition manager using the provided channel */
        PartitionManager(Channel* ch);

    private:

};

/**
 * A channel for working safely with partitions. In short, this channel wraps
 * a raw device channel and makes sure every read, write and search is relative
 * to the partition offset, as well as honering the partition length.
 */
class PartitionChannel : public Channel
{
    public:

        /** C'tor */
        PartitionChannel(AbstractChannel* deviceChannel, fpos_t offset, fpos_t capacity)
        {
            this->deviceChannel = deviceChannel;
            this->offset = offset;
            this->capacity = capacity;
            this->position = 0;
        }

        /** D'tor */
        virtual ~PartitionChannel()
        {
            // Remove the clone
            delete deviceChannel;
        }

        /* Implements the Channel interface */

        virtual AbstractChannel* clone()
        {
            PartitionChannel* copy =
                new PartitionChannel(deviceChannel, offset, capacity);
            *copy = *this;

            copy->deviceChannel = deviceChannel->clone();
            copy->setPosition(0);

            return copy;
        }

        /** See Channel#setPosition */
        virtual error_t setPosition(fpos_t pos)
        {
            if(capacity == 0 || pos < capacity)
            {
                this->position = pos;
                return deviceChannel->setPosition(offset + pos);
            }
            else
                return E_RANGE;
        }

        /** See Channel#read */
        virtual ssize_t read(uint8* bytes, size_t length, bool block = true)
        {
            ssize_t res;

            // A partition needs to be read before the capacity is know,
            // so we accept zero capacity.
            if(capacity == 0 || (position + length) < capacity)
            {
                // Resync device channel position (which is shared / not cloned)
                //setPosition(position);

                res = deviceChannel->read(bytes, length, block);
                position = deviceChannel->getPosition() - offset;
            }
            else
            {
                res = -E_RANGE;
            }

            return res;
        }

        /** See Channel#write */
        virtual ssize_t write(const uint8* bytes, size_t length, bool block = true)
        {
            ssize_t res;

            if((position + length) < capacity)
            {
                // Resync device channel position (which is shared / not cloned)
                //setPosition(position);

                res = deviceChannel->write(bytes, length, block);
                position = deviceChannel->getPosition() - offset;
            }
            else
            {
                res = -E_RANGE;
            }

            return res;
        }

    private:

        /** Partition offset; all channel ops are relative to this */
        fpos_t offset;

        /** The underlying raw device channel */
        AbstractChannel* deviceChannel;
};

#endif //  _APOX_DRIVERS_FS_GENERIC_PARTITIONS_H_
