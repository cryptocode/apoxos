/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#include <kernel/drivers/ata/ATADriver.h>
#include <kernel/util/Autoptr.h>
#include <kernel/util/Math.h>
#include <kernel/threads/Thread.h>
#include <kernel/adt/Buffer.h>
#include <kernel/Errors.h>

/**
 * C'tor
 *
 * @param driver The driver instance we're providing a Channel on
 */
ATAIOChannel::ATAIOChannel(ATADriver* driver)
{
    this->driver = driver;
}

/* See declaration */
AbstractChannel* ATAIOChannel::clone()
{
    ATAIOChannel* copy = new ATAIOChannel(driver);
    *copy = *this;
    copy->setPosition(0);

    return copy;
}

/**
 * This sets the physical disk position for this channel.
 */
error_t ATAIOChannel::setPosition(fpos_t position)
{
    // Allow zero-capacity, since we need to read in order to obtain the capacity.
    if(capacity == 0 || position < capacity)
    {
        //out << "[ATAIO] setPosition(" << position << ")" << eol;
        this->position = position;
        return E_OK;
    }
    else
    {
        out << "[ATAIO] Out of range" << eol;
        return Thread::setLastError(E_RANGE);
    }
}

/**
 * Write 'length' bytes on the disk and adjusts the current channel position.
 *
 * @returns Number of bytes written or a negated error value
 * @block If true, block until all bytes have been written. Default is false.
 */
ssize_t ATAIOChannel::write(const uint8* bytes, size_t length, bool block /*=true*/)
{
    if(position + length > capacity)
    {
        assert(false);
        return -E_RANGE;
    }

    int res = length;

    uint64 lbaStart = position / (uint64)driver->getSectorSize();
    uint64 lbaEnd = (position + length) / (uint64)driver->getSectorSize();
    uint64 lbaEndExtra = (position + length) % (uint64)driver->getSectorSize();
    if(lbaEndExtra)
        lbaEnd++;

    assert(lbaStart <= lbaEnd);

    size_t sectorCount = size_t(lbaEnd - lbaStart);
    sectorCount = Math::max(sectorCount, size_t(1));
    Buffer buf(sectorCount * driver->getSectorSize());

    // This is just a first stupid impl: we read *all* the affected sectors,
    // update the relevant portion and write the sectors back. A better impl is
    // to just read the first and the last sectors (which may be partially
    // affected), and blindly overwrite the middle-sectors.
    res = driver->readSectors(lbaStart, sectorCount, buf);
    if(res != E_OK)
    {
        return -(Thread::setLastError(res));
    }

    // Update the sector buffer
    int sectorOffset = position % driver->getSectorSize();
    memcpy(buf.data() + sectorOffset, bytes, length);

    // Write back the sectors
    res = driver->writeSectors(lbaStart, sectorCount, buf);
    if(res != E_OK)
    {
        return -(Thread::setLastError(res));
    }

    // Clean up and adjust position
    position += length;

    // We always read everything; i.e. we're blocking
    return length;
}

/**
 * Read bytes from the disk and adjust the current channel position.
 *
 * @return Number of bytes read, or negated error value
 */
ssize_t ATAIOChannel::read(uint8* outBytes, size_t length, bool block /*=true*/)
{
    if(position + length > capacity)
    {
        assert(false);
        return -E_RANGE;
    }

    ssize_t res = (ssize_t)length;

    uint64 lbaStart = position / (uint64)driver->getSectorSize();
    uint64 lbaEnd = (position + length) / (uint64)driver->getSectorSize();
    uint64 lbaEndExtra = (position + length) % (uint64)driver->getSectorSize();
    if(lbaEndExtra)
        lbaEnd++;

    assert(lbaStart <= lbaEnd);

    size_t sectorCount = size_t(lbaEnd - lbaStart);
    sectorCount = Math::max(sectorCount, size_t(1));
    Buffer buf(sectorCount * driver->getSectorSize());
    assert (sectorCount > 0);

    res = driver->readSectors(lbaStart, sectorCount, buf);
    if(res != E_OK)
    {
        return -Thread::setLastError(res);
    }

    // Copy the relevant portion. For reasonably small buffers, it's probably faster
    // to just read all sectors at once (rather than several read rountrips) and copy
    // the portion to the output buffer. TODO: THIS MUST BE PROFILED!
    int sectorOffset = position % driver->getSectorSize();
    memcpy(outBytes, buf.data() + sectorOffset, length);

    // Adjust position
    position += length;

    // We always read everything; i.e. we're blocking
    return length;
}
