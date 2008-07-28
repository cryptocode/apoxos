/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 */

/**
 * \file
 * Namespace node implementation for FAT files and directories
 */

#include <kernel/drivers/fs/fat/FATNode.h>
#include <kernel/drivers/fs/fat/FAT.h>
#include <kernel/drivers/fs/fat/FATFileChannel.h>

/* See declaration */
FATNode::FATNode(FATDriver* driver, uint32 startCluster)
{
    this->driver = driver;
    this->channel = null;
    this->startCluster = startCluster;
    this->size = 0;
    this->isDirectory = false;
}

/* See declaration */
FATNode::~FATNode()
{
    // Note that ~Node deallocates children
}

/* See declaration */
Object* FATNode::getInterface(int IID)
{
    if(IID == IIDChannel && !isDirectory)
    {
        FATFileChannel* ch = new FATFileChannel(this);
        ch->setResourceManager(this);
        ch->setCapacity(getSize());
        ch->setPosition(0);

        return ch;
    }
    else if(IID == IIDFile)
    {
        return static_cast<IFile*>(this);
    }
    else
    {
        return null;
    }
}

/* See declaration */
error_t FATNode::release(Object* obj)
{
    if(obj)
    {
        delete (FATFileChannel*) obj;
    }

    return E_OK;
}

/** @see IFileInfo#create */
error_t FATNode::create(const String& name, uint64 attr,
                        Node*& node, bool directory)
{
    return driver->create(this, name, attr, (FATNode*&)node, directory);
}

/** @see IFileInfo#setSize */
error_t FATNode::setSize(fpos_t newSize)
{
    error_t res = E_OK;

    // TODO: file size is dependant on FAT-type -> add to Fatoid
    if(newSize <= UINT32_MAX)
    {
        res = driver->setLength(this, (uint32)newSize);

        if(res == E_OK)
            size = newSize;
    }
    else
    {
        res = E_RANGE;
    }

    return res;
}

/* See declaration */
List<Node*>* FATNode::getChildren()
{
    if(isDirectory)
    {
        // Empty
        removeChildren();

        // This populates our child array
        driver->lock();
        driver->scanDirectory(this->startCluster, this);
        driver->unlock();

        return Node::children;
    }
    else
        return null;
}

/* See declaration */
Node* FATNode::getChildByName(const String& name)
{
    FATNode* res = (FATNode*)Node::getChildByName(name);

    if(res == null)
    {
        driver->lock();
        driver->scanDirectory(startCluster, NULL, (String*)&name, &res);
        driver->unlock();

        if(res)
            addChild(res);
        //else
        //    out << name << " path component was NOT found (FATNode)" << eol;
    }

    return res;
}

/* See declaration */
error_t FATNode::removeChild(Node* child)
{
    error_t res = E_INVALID;

    if(child)
    {
        driver->lock();

        // Delete the file itself and the in-memory node
        res = driver->remove((FATNode*)child);
        Node::removeChild(child);
        driver->unlock();
    }

    return res;
}

/* See declaration */
error_t FATNode::rename(const String& newname)
{
    error_t res = driver->rename(this, newname);

    // If the rename succeeded, update the in-memory node
    if(res == E_OK)
        Node::rename(name);

    return res;
}
