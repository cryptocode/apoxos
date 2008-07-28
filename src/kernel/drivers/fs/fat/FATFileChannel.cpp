/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 */

#include <kernel/drivers/fs/fat/FATFileChannel.h>
#include <kernel/drivers/fs/fat/FATNode.h>
#include <kernel/drivers/fs/fat/FAT.h>
#include <kernel/threads/Thread.h>
#include <kernel/adt/Buffer.h>

FATFileChannel::FATFileChannel(FATNode* node)
{
    this->node = node;
}

FATFileChannel::~FATFileChannel()
{
}

/* See declaration */
AbstractChannel* FATFileChannel::clone()
{
    assert(false);

    FATFileChannel* copy = new FATFileChannel(node);
    *copy = *this;
    copy->setPosition(0);

    return copy;
}

/* See declaration */
ssize_t FATFileChannel::read(uint8* bytes, size_t length, bool block)
{
    if(length == 0)
        return 0;

    // TODO: xfer must return (in a param?) the number of bytes read, since
    // the caller may request > file size

    Buffer buf((const void*)bytes, length);
    assert(length > 0);
    assert(buf.capacity() == length);

    //printf(" * * * FAT: READING '%S' FROM CLUSTER %u, CHANNEL POS IS %Lu, LEN %d\n",
    //       &node->getName(), node->startCluster, position, length);


    //flag_t flags;
    //CPU::localIrqOff(&flags);

    // LOCKING HERE HAS CAUSED DEADLOCKS BECAUSE XFER MAKES A PAGE FAULT WHICH
    // NEEDS AN XFER, ETC, ETC.
    //node->driver->lock();
    size_t read = 0;
    error_t err = node->driver->xfer(node, position, buf, false, false, read);
    //node->driver->unlock();

    //CPU::localIrqRestore(flags);

    //printf("    ACTUALLY READ: %u\n", length);

    // We expect the caller to try to read more than we can offer; in which
    // case we set pos at the end of the channel.
    if(err == E_RANGE)
    {
        fprintf(stderr, "E_RANGE ERROR. SETTING POSITION TO %Lu\n", getCapacity()-1);
        setPosition(getCapacity()-1);
        err = E_OK;
    }
    else if(err == E_OK)
    {
        this->setPosition(getPosition() + length);
    }
    else
    {
        fprintf(stderr, "[FAT] ERROR: %d\n", err);
        Thread::setLastError(err);
    }

    printf("FATFileChannel::read. Requested: %u, Actually read: %u\n", length, read);
    return err != E_OK ? -1 : read;
}

/* See declaration */
int FATFileChannel::read()
{
    if(getPosition()+1 == getCapacity())
        return -1;

    Buffer buf(1);

    //node->driver->lock();
    size_t read;
    error_t err = node->driver->xfer(node, position, buf, false, false, read);
    //node->driver->unlock();

    if(err == E_OK)
        setPosition(getPosition() + 1);
    else
    {
        fprintf(stderr, "[FAT] READ() ERROR: %d\n", err);
        Thread::setLastError(err);
    }

    return err != E_OK ? -1 : (int)buf.data()[0];
}

/* See declaration */
ssize_t FATFileChannel::write(const uint8* bytes, size_t length, bool block)
{
    if(length == 0)
        return 0;

    Buffer buf((const void*)bytes, length);
    assert(length > 0);
    assert(buf.capacity() == (size_t)length);

    //printf(" * * * FAT: WRITING '%S' TO CLUSTER %u, CHANNEL POS IS %Lu, LEN %d, BUFF BEFORE: 0x%x\n",
    //       &node->getName(), node->startCluster, position, length, bytes);

    //node->driver->lock();
    size_t written;
    error_t err = node->driver->xfer(node, position, buf, true, true, written);

    //printf(" * * * FAT: XFER'ED: %u, RES=%d\n", length, err);

    // The file may have grown
    setCapacity(node->getSize());

    //node->driver->unlock();

    if(err == E_RANGE)
    {
        setPosition(getCapacity()-1);
        err = E_OK;
    }
    else if(err == E_OK)
        this->setPosition(getPosition() + length);
    else
    {
        printf(" * * * ERROR: %d\n", err);
        Thread::setLastError(err);
    }

    return err != E_OK ? -1 : written;
}

/* See declaration */
int FATFileChannel::write(int c)
{
    Buffer buf(1);
    *buf.data() = (uint8)c;

    //node->driver->lock();
    size_t written;
    error_t err = node->driver->xfer(node, position, buf, true, true, written);

    // The file may have grown
    setCapacity(node->getSize());

    //node->driver->unlock();

    /*if(err == E_RANGE)
        setPosition(getCapacity()-1);
    else */if(err == E_OK)
    {
        this->setPosition(getPosition() + 1);
    }
    else
        Thread::setLastError(err);

    return err != E_OK ? -1 : (int)buf.data()[0];
}
