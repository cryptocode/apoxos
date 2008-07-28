/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 */

#ifndef _APOX_FS_FAT_FATNODE_H_
#define _APOX_FS_FAT_FATNODE_H_

#include <kernel/libc/std.h>
#include <kernel/Node.h>
#include <lib/api/IFileInfo.h>

class FATDriver;
class FATFileChannel;

/**
 * Implements access to a FAT file or directory
 */
class FATNode : public Node, public IFile, public ResourceManager
{
    friend class FATDriver;
    friend class FATFileChannel;

    public:

        /**
         * C'tor
         *
         * @param driver The supporting driver
         */
        FATNode(FATDriver* driver, uint32 startCluster);

        /**
         * Destructor removes child nodes
         */
        virtual ~FATNode();

    /* Node overrides */
    public:

        /**
         * Renames the file
         *
         * @see Node#rename
         * @see FATDriver#rename
         */
        virtual error_t rename(const String& name);

        /**
         * FATNode returns children non-recursively, but that's not a problem
         * since doing getChildren()->getChildren(), etc will cause FATNode's
         * version to be called (and as such, does load the tree on demand)
         *
         * @see Node#getChildren
         */
        virtual List<Node*>* getChildren();

        /**
         * This will remove a file (TODO: or an empty directory)
         *
         * @param child Child to remove
         * @return error_t E_NOT_FOUND, E_NOT_EMPTY, E_*
         * @see Node#removeChild
         */
        virtual error_t removeChild(Node* child);

        /**
         * This override first calls the base version in case it's already
         * in the child list. If not, it calls the FAT driver which will scan
         * the directory for an entry with the given name.
         *
         * @param name Name of entry. Currently, only 8.3 short names works.
         * @return Node* null if not found
         * @see Node#getChildByName
         */
        virtual Node* getChildByName(const String& name);

        /** Honor requests for IIDChannel */
        virtual Object* getInterface(int IID);

        /** @see ResourceManager#release */
        virtual error_t release(Object* obj);


    /* IFile implementation */
    public:

        /** @see IFileInfo#create */
        virtual error_t create(const String& name, uint64 attr,
                               Node*& node, bool directory);

        /** @see IFileInfo#setSize */
        virtual error_t setSize(fpos_t newSize);

        /** @see IFileInfo#getSize */
        virtual fpos_t getSize()
        {
            return size;
        }

        /** @see IFileInfo */
        virtual error_t setAttributes(uint64 attr)
        {
            this->attr = attr;
            return E_OK;
        }

        /** @see IFileInfo */
        virtual uint64 getAttributes()
        {
            return attr;
        }

        /** @see IFileInfo */
        virtual bool getIsDirectory()
        {
            return isDirectory;
        }

        /** @see IFileInfo */
        virtual bool getIsHidden()
        {
            return isHidden;
        }

        /** @see IFileInfo */
        virtual bool getIsSystem()
        {
            return isSystem;
        }

        /** @see IFileInfo */
        virtual fpos_t getStartBlock()
        {
            return startCluster;
        }

    private:

        /** Size, in bytes */
        fpos_t size;
        uint64 attr;

        // TODO: encode these in 'attr' in a way that's standard across VFS's
        bool isDirectory;
        bool isHidden;
        bool isSystem;

        /** FAT driver backptr */
        FATDriver* driver;

        /** The channel */
        FATFileChannel* channel;

        /** Start cluster. If 0, the node is assumed to be the root directory */
        uint32 startCluster;

        /**
         * This is the byte position, relative to the beginning of the volume,
         * of the short directory entry.
         */
        uint32 shortEntryOffset;
};

#endif // _APOX_FS_FAT_FATNODE_H_
