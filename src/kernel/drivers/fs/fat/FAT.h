/*
 * Apox Operating System
 * Copyright (c) 2006, 2007, cryptocode
 */

#ifndef _APOX_DRIVERS_FS_FAT_
#define _APOX_DRIVERS_FS_FAT_

#include <stdio.h>
#include <kernel/drivers/fs/FileSystem.h>
#include <kernel/io/Channel.h>

// Acceptable values are 512, 1024, 2048 and 4096
#define FAT_BOOTSECTOR_SIZE     512
#define FAT_VOLUMEID_LEN        11

class FATNode;
class Buffer;

/** A span of free clusters */
struct FreeEntry
{
    FreeEntry(uint32 startCluster, uint32 count)
    {
        this->startCluster = startCluster;
        this->count = count;
    }

    uint32 startCluster;
    uint32 count;
};

/**
 * FAT file system driver
 *
 * This driver supports FAT12, FAT16 and FAT32 on both little and big endian
 * machines. FAT is a rather simple (and very clumsy) file system, but it is
 * used on anything from desktops to memory sticks and floppies, so it's a must.
 * It also a nice way to integrate with other OS's (even in emulators by means
 * of loopback devices.)
 *
 * The main reference source is [S 3]
 */
class FATDriver : public Filesystem, public ResourceManager
{
    friend class FATNode;

    public:

        /** FAT type. These enums are indices into tables, so do NOT change the values */
        enum FATType
        { FAT12=0, FAT16=1, FAT32=2 };

        /**
         * Creates and inits a new FAT instance
         *
         * @returns NULL if the driver could not be created
         */
        static FATDriver* initialize(Channel* ch);

        /** Returns the volume name (up to 11 characters) */
        char* getVolumeName()
        {
            return volumeId;
        }

    /** Overrides */
    public:

        /** @see FileSystem#mount */
        virtual error_t mount(Node* parent);

        /**
         * Support IIDChannelID
         *
         * TODO: should be able to ask ATADevice the same thing
         */
        virtual Object* getInterface(int IID)
        {
            Object* res = null;

            if(IID == IIDChannel)
            {
                AbstractChannel* clone = channel->clone();
                res = clone;
            }

            return res;
        }

        /** @see ResourceManager#release */
        error_t release(Object* obj)
        {
            if(obj != null)
            {
                delete (Channel*)obj;
            }

            return E_OK;
        }

        /**
         * C'tor
         * @param ch A channel on which the file system resides (usually a PartitionChannel, but could be anything)
         */
        FATDriver(Channel* ch);

        /**
         * This function operates in three modes:
         *
         * 1) If search is null (default), all directory entries are read
         *    the parentNode's children are populated with FATNode objects.
         *
         * 2) If search is non-null and a matching entry is found, 'target' is
         *    set to a newly allocated FATNode describing the matched entry.
         *    The 'search' argument must be in 8.3 notation, i.e. "READ.ME"
         *    (long-name search is not yet supported.
         *
         * 3) If both search and parentNode is null, scanDirectory will look
         *    for a free directory entry. If required, a new cluster will be
         *    allocated for the entry. 'target' must point to an existing
         *    FATNode with a valid parent pointer, so the directory from which
         *    to find a free entry can be searched.
         *
         * @param cluster Cluster number of directory. If 0, the FAT12/16 root
         *                directory is scanned (FAT32 uses an arbitrary cluster)
         * @param parentNode If not null, the children collection will be populated
         * @param search If not null, search for this directory
         * @param target If 'search' is not null and there is a match, target
         *               will point to the matching Node. If both 'search' and
         *               'target' is null, it will point to a new node for the
         *               directory entry.
         */
        error_t scanDirectory(uint32 cluster, Node* parentNode,
                              String* search = null, FATNode** target = null);

        /**
         * Low-level transfers of buffer to or from file.
         *
         * When writing: If the file is to small and expand is false, xfer fails
         * with E_RANGE. If expand is true, the file is automatically expanded,
         * but xfer may return E_DEVICE_FULL if the volume limit is reached.
         *
         * Non-growbable files are useful to safely implement virtual disk
         * partitions.
         *
         * On error, an error_t value is returned. The threads lastError value
         * is also updated to the same value.
         *
         * @param startBlock To FAT, this is the starting cluster of file
         * @param node FATNode representing the file to xfer on
         * @param offset Offset into file from where reading or writing should start
         * @param buf Buffer to read into or write from
         * @param write If true, write
         * @param expand If both write and expand is true, the file will be expanded if necessary
         * @param xfered Number of bytes transferred
         *
         * @return error_t E_DEVICE_ERROR (bad clusters), E_RANGE, E_DEVICE_FULL
         */
        error_t xfer(FATNode* node, fpos_t offset, Buffer& buffer,
                     bool write, bool expand, size_t& xfered);

        /**
         * this must override something...
         *
         * Creates a new file
         *
         * @param parent
         * @param name Short name (long names not supported yet.)
         * @param directory If true, create a directory
         *
         * @return error_t E_DEVICE_FULL, E_*
         */
        error_t create(FATNode* parent, const String& name,
                       uint64 attr, FATNode*& created, bool directory);

        /**
         * Removes the file or directory. The directory will only be removed
         * if it is empty.
         *
         * @param node Node to remove
         * @return E_*
         */
        error_t remove(FATNode* node);


        /**
         * Rename the file. The caller is assumed to already have checked that
         * the new name is non-existant.
         *
         * @param node Represents the file to rename
         * @return error_t E_*
         */
        error_t rename(FATNode* node, const String& newName);

        /**
         * Sets the new length of the file. If 0, it truncates the file.
         *
         * @param node Node for file to truncate
         * @param newsize The new size of the file
         *
         * @return E_OK, E_RANGE (end of volume space)
         */
        error_t setLength(FATNode* node, size_t newsize);

    protected:

        /**
         * Get next cluster index from the FAT
         *
         * @param cluster Current cluster index, 0 on error
         */
        uint32 getNextCluster(uint32 cluster);

        /**
         * This updates the FAT table. Note that this terminates the chain
         * by writing EOC into nextCluster's FAT entry (unless nextCluster is
         * EOC)
         *
         * @param cluster Cluster entry to update
         * @param nextCluster The next cluster
         * @return E_*
         */
        error_t updateNextCluster(uint32 cluster, uint32 nextCluster);

        /** Given a cluster index, get the corresponding entry index in the FAT */
        uint32 getFATEntry(uint32 cluster);

        /** Sector on which the FAT entry for the given cluster resides */
        uint32 getFATEntrySector(uint32 cluster);

        /** Offset on sector in which the FAT entry for the given cluster resides */
        uint32 getFATEntryOffset(uint32 cluster);

        /**
         * DEBUG: Transfer data
         *
         * @return E_DEVICE_ERROR (bad sectors), E_RANGE (hitting end-of-cluster)
         */
        error_t readFile(uint32 cluster, uint32 remainingBytes, uint8* buf);

        /** DEBUG: load ELF */
        void loadUserland(uint32 startCluster, uint32 filesize);

        /** Given a cluster, compute and return lba */
        uint32 cluster2lba(uint32 cluster);

        /** Given a cluster, compute and return byte position */
        uint32 cluster2pos(uint32 cluster);

        /**
         * Scans the FAT table and stores the free clusters in a compact RLE-
         * type list. When the FAT is highly fragmented, the freelist may take
         * a considerable amount of memory.
         *
         * This function is called:
         *
         * 1) When initializing the driver
         * 2) After freeing clusters (by a separate thread?)
         *
         * @return error_t Any errors returned by Channel methods.
         */
        error_t readFreeClusters();

        /** Helper for readFreeClusters */
        uint32 getFATEntry(Buffer& fatchunk, int index);

        /**
         * Get the next available free cluster.
         *
         * @return 0 if there are no clusters available
         */
        uint32 getFreeCluster();


    private:

        /** List of free cluster spans */
        List<FreeEntry*> freeEntries;

        /** A channel accessing the FAT volume */
        Channel* channel;

        /** Bytes per sector (almost always 512) */
        uint16 bytesPerSect;

        /** Bytes per cluster (sectPerCluster * bytesPerSect) */
        uint32 bytesPerCluster;

        /** Sectors per cluster */
        uint8 sectPerCluster;

        /** Sectors in the reserved region */
        uint16 reservedSects;

        /** Number of FATs on the volume (almost always 2) */
        uint8 numFATs;

        /** Number of root directory entries on FAT12 and FAT16 volumes */
        uint16 rootEntries;

        /** Total number of sectors on the volume */
        uint32 totSectors;

        /** Size of *one* FAT table, in sectors */
        uint32 sizeFAT;

        /** Number of sectors in the data region */
        uint32 dataSectors;

        /** Cluster count */
        uint32 numClusters;

        /**
         * Sector count in root dir; this is computed as
         *
         *  ((rootEntries * 32) * (bytesPerSect-1)) / bytesPerSect
         */
        uint32 rootDirSectors;

        /** FAT32 only: First cluster of the root directory (usually 2) */
        uint32 rootCluster;

        /** The FAT is mirrored? */
        bool mirrored;

        /** If mirrored == false, this contains the 0-based number of the active FAT */
        int activeFAT;

        /** Volume name (0-terminated) */
        char volumeId[FAT_VOLUMEID_LEN+1];

        /** FAT type (FAT12=0,FAT16=1, FAT32=2) */
        FATType type;

        /** BIOS Parameter Block; basically the FAT volume's own bootsector */
        uint8 BPB[FAT_BOOTSECTOR_SIZE];

        /** This will contain a cluster worth of 0's, used to wipe out
         * newly allocated clusters. */
        uint8* nullCluster;
};

#endif // _APOX_DRIVERS_FS_FAT_
