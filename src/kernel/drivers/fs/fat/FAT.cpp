/*
 * Apox Operating System
 * Copyright (c) 2006, 2007, cryptocode
 */

#include <assert.h>
#include <kernel/Kernel.h>
#include <kernel/drivers/fs/fat/FAT.h>
#include <kernel/drivers/fs/fat/FATNode.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/threads/Thread.h>
#include <kernel/util/Convert.h>
#include <kernel/util/Math.h>
#include <kernel/util/Hexdump.h>
#include <kernel/adt/Buffer.h>

namespace
{
    #define FAT_DEBUG_LEVEL                         0
    #define LOG_PREFIX                              "[FAT] "

    /* File and directory attributes */

    #define FAT_ATTR_READ_ONLY   	                0x01
    #define FAT_ATTR_HIDDEN 		                0x02
    #define FAT_ATTR_SYSTEM 		                0x04
    #define FAT_ATTR_VOLUMEID 	                    0x08
    #define FAT_ATTR_DIRECTORY		                0x10
    #define FAT_ATTR_ARCHIVE  		                0x20
    #define FAT_ATTR_LONG_TEXT		                0x0F
    #define FAT_ATTR_ORDINAL_MASK	                0x3F
    #define FAT_ATTR_LONG_TEXT_LAST	                0x40

    /* Directory entry offsets */
    #define FAT_DIRENT_IDX_NAME                     0
    #define FAT_DIRENT_IDX_ATTR                     11

    /** In a long entry, idx 13 is the checksum */
    #define FAT_DIRENT_IDX_CHECKSUM                 13

    /** In a short entry, idx 13 is millsecond stamp */
    #define FAT_DIRENT_IDX_TIME_CREATE_TENTH        13
    #define FAT_DIRENT_IDX_TIME_CREATE              14
    #define FAT_DIRENT_IDX_DATE_CREATE              16
    #define FAT_DIRENT_IDX_DATE_LASTACCESS          18

    /** High word of cluster number; always 0 for FAT12/16 */
    #define FAT_DIRENT_IDX_CLUSTER_HI               20
    #define FAT_DIRENT_IDX_TIME_LASTWRITE           22
    #define FAT_DIRENT_IDX_DATE_LASTWRITE           24

    /** Low word of cluster number */
    #define FAT_DIRENT_IDX_CLUSTER_LO               26

    /** 32-bit file size in bytes */
    #define FAT_DIRENT_IDX_FILESIZE                 28

    /** Size of each directory entry */
    #define FAT_DIRENT_SIZE                         32

    /* Dir entry name[0] constants */

    /** Directory entry is free */
    #define FAT_DIRENT_FREE                         0xE5

    /**
     * Directory entry is free and there are no more entries.
     */
    #define FAT_DIRENT_FREE_END                     0x00

    /**
     * If name[0] == 0x05, then it should be interpreted as 0xE5; this is
     * due to a unicode screwup in FAT, since 0xE5 is a valid kanji lead byte
     */
    #define FAT_DIRENT_KANJI_LEAD                   0x05

    /**
     * We collect a number of useful factoids on the three FAT formats
     * in a table for fast'n'easy access.
     *
     * FAT + Factoid = Fatoid
     */
    struct Fatoid
    {
        /** FAT table entry size in bits */
        int8 fatBits;

        /**
         * End Of Clusterchain (EOC) mark. This is the lower bound; higher values
         * also indicate EOC.
         */
        uint32 eocMark;

        /**
         * Addressing mask, i.e. only lower 28 bits used for FAT32.
         * This also functions as the end-of-chain mark after masking the lba
         * with lbaMask.
         */
        uint32 lbaMask;

        /** The FAT entry signature of bad sectors */
        uint32 badClusterMark;
    };

    /**
     * Note that the enum values FAT12, FAT16 and FAT32 are also indices into
     * this table, so you can do facts[type].member;
     */
    Fatoid facts[3] =
    {
        /* FAT12 */
        { 12, 0x00000FF8, 0xFFFFFFFF /*?*/, 0xFF7},

        /* FAT16 */
        { 16, 0x0000FFF8, 0xFFFFFFFF /*?*/, 0xFFF7},

        /* FAT32 */
        { 32, 0x0FFFFFF8, 0x0FFFFFFF /* 28 bits usable for lba addressing */ , 0x0FFFFFF7}
    };

    /**
     * This does what the spec says: computes an 8 bit checksum of the 11 byte
     * array. It is called on every directory long-name entry to handle the case
     * where the volume is upgraded from one FAT version to another. If the
     * checksum check fails for any of the long-name entries, the entry is to
     * be treated as an orphan.
     *
     * @param shortEntry 11-byte short name from a long entry
     * @return uint8 Checksum
     */
    uint8 checksum(uint8* shortEntry)
    {
        uint8 res = 0;

        for(int i=11; i>0; i--)
        {
            res = ((res & 1) ? 0x80 : 0) + (res >> 1) + *shortEntry++;
        }

        return res;
    }

    /**
     * Converts from internal FAT format to trimmed 8.3 format.
     *
     * I.e. "READ    ME " => "READ.ME"
     */
    void collapseShortName(char* name)
    {
        char pre[9];
        char suff[4];

        memcpy(pre, name, 8);
        memcpy(suff, name+8, 3);

        name[0] = '\0';

        for(int i=7; i >= 0; i--)
        {
            if(pre[i] != ' ')
            {
                pre[i+1] = '\0';
                break;
            }
            else
                pre[i] = '\0';
        }

        for(int i=2; i >= 0; i--)
        {
            if(suff[i] != ' ')
            {
                suff[i+1] = '\0';
                break;
            }
            else
                suff[i] = '\0';
        }

        strcpy(name, pre);
        if(strlen(suff))
        {
            strcat(name, ".");
            strcat(name, suff);
        }
    }

    /**
     * Converts a short name like "README.TXT" to the internal FAT name, like
     * "README  TXT"
     *
     * @param name Name to explode. The input must be in correct 8.3 format
     * @param exploded An array of at least 11 bytes
     */
    void explode(const String& name, uint8* exploded)
    {
        int index = 0;
        unichar_t* chars = name.getCharacters();

        for(size_t i=0; i < name.size(); i++)
        {
            // Reached the extension?
            if(chars[i] == L'.')
            {
                // Add spaces to the first name
                for(; index < 8; index++)
                    exploded[index] = ' ';

                i++;
            }

            exploded[index++] = (uint8)chars[i];
        }

        // Add spaces to extension
        for(; index < 11; index++)
            exploded[index] = ' ';
    }

}// anon namespace

/**
 * Private c'tor
 * @param ch A channel on which the file system resides (usually a PartitionChannel, but could be anything)
 */
FATDriver::FATDriver(Channel* channel)
{
    this->channel = channel;
    this->channel->setResourceManager(this);

    mirrored = true;
    volumeId[0] = 0;
}

/**
 * Creates and initalizes a new FAT driver instance
 *
 * We check the FAT signature and collect essential info from the BIOS param
 * block that we need later.
 */
FATDriver* FATDriver::initialize(Channel* channel)
{
    FATDriver* driver = new FATDriver(channel);
    uint8* BPB = driver->BPB;

    channel->setPosition(0);
    channel->read(BPB, FAT_BOOTSECTOR_SIZE);

    // Handle invalid partitions
    if(BPB[0x1FE] != 0x55 || BPB[0x1FF] != 0xAA)
    {
        log << LOG_PREFIX << "Not a FAT device" << eol;
        delete driver;
        driver = null;
    }
    else
    {
        // FACTOR OUT THIS BLOCK

        driver->bytesPerSect = Convert::le2host((uint16*)&BPB[11]);
        driver->sectPerCluster = BPB[13];
        driver->bytesPerCluster = driver->bytesPerSect * driver->sectPerCluster;
        driver->reservedSects = Convert::le2host((uint16*)&BPB[14]);
        driver->numFATs = BPB[16];
        driver->nullCluster = new uint8[driver->bytesPerCluster];
        memset(driver->nullCluster, 0, driver->bytesPerCluster);

        // This is for FAT12 and FAT16 only as FAT32 stores the root dir as a
        // cluster chain
        driver->rootEntries = Convert::le2host((uint16*)&BPB[17]);

        // Get total number of sectors on the volume (all four FAT regions).
        // There are two fields in the BPB for this. If the first one is 0, the
        // second one applies (the second one is always used by FAT32).
        driver->totSectors = Convert::le2host((uint16*)&BPB[19]);
        if(driver->totSectors == 0)
        {
            driver->totSectors = Convert::le2host((uint32*)&BPB[32]);

            if(driver->totSectors == 0)
            {
                // while debugging...
                assert(false);

                Thread::current()->setLastError(E_DEVICE_ERROR);
                return null;
            }
        }

        // Get size of *one* FAT table; sector count. As with sectors, there are
        // two fields in the BPB for this.
        driver->sizeFAT = Convert::le2host((uint16*)&BPB[22]);
        if(driver->sizeFAT == 0)
        {
            driver->sizeFAT = Convert::le2host((uint32*)&BPB[36]);
            assert(driver->sizeFAT != 0);
        }

        // This, according to the spec, is the one and only way to determine the
        // FAT type is by looking at the sector count. Basically, we compute the
        // number of sectors on the data region and use that to figure out the
        // actual FAT subtype (12, 16, 32).

        // First of all, compute sector count of the root dir
        driver->rootDirSectors = ((driver->rootEntries * 32) *
                (driver->bytesPerSect-1)) / driver->bytesPerSect;

        // Then we determine sector count in the data region
        driver->dataSectors = driver->totSectors - (driver->reservedSects +
                (driver->numFATs * driver->sizeFAT) + driver->rootDirSectors);

        // Compute number of clusters in the data region
        driver->numClusters = driver->dataSectors / driver->sectPerCluster;

        // Now we have the required insight to determine the FAT type
        if(driver->numClusters < 4085)
        {
            driver->type = FAT12;
        }
        else if(driver->numClusters < 65525)
        {
            driver->type = FAT16;
        }
        else
        {
            driver->type = FAT32;
        }

        // Mirrored FAT?
        if(driver->type == FAT32)
        {
            uint16 extFlags = Convert::le2host((uint16*)&BPB[40]);
            //printf("EXT FLAGS:%u\n", (uint32)extFlags);

            if(Bits::isSet(extFlags, 7))
            {
                driver->mirrored = false;

                // Bits [3:0] contains 0-based number of the active fat
                driver->activeFAT = extFlags & 0xF;
            }
        }

        // Root directory cluster (usually 2). This is how FAT32 access the root
        // directory; just another cluster chain.
        if(driver->type == FAT32)
        {
            driver->rootCluster = Convert::le2host(*(uint32*)&BPB[44]);
        }
        else
        {
            // In FAT12/16, the root directory is located at a fixed position
            driver->rootCluster = 0;
        }

        // Get volume label; this may be overriden by a special root directory
        // entry later on
        driver->volumeId[11] = '\0';
        int volOffset = (driver->type == FAT32) ? 71 : 43;
        memcpy(driver->volumeId, (const void*)&BPB[volOffset], FAT_VOLUMEID_LEN);

        //
        // Print what we gathered
        //
        log << "Volume name              : " << driver->volumeId << eol;
        log << "Bytes per sector         : " << driver->bytesPerSect << eol;
        log << "Sectors per cluster      : " << driver->sectPerCluster << eol;
        log << "Cluster size, in bytes   : " << driver->bytesPerCluster << eol;
        log << "Reserved sectors         : " << driver->reservedSects << eol;
        log << "Number of FAT's          : " << driver->numFATs << eol;
        log << "Total sectors            : " << driver->totSectors << " ("
                                             << ((uint64)driver->totSectors*driver->bytesPerSect)
                                             << " bytes)" << eol;
        log << "FAT table size, sectors  : " << driver->sizeFAT << eol;

        if(!driver->mirrored)
            log << "Non-mirrored. Active FAT : " << driver->activeFAT << eol;
        else
            log << "Mirrored FAT" << eol;

        if(driver->type == FAT32)
        {
            // FAT32 stores the root dir in a normal cluster chain
            log << "Root directory cluster   : " << driver->rootCluster << eol;
        }
        else
        {
            // FAT12/16 has a special Root Directory Region
            log << "Root entries             : " << driver->rootEntries << eol;
            log << "Root directory size      : " << driver->rootEntries*32 << eol;
        }

        /* Register device */

        String name(driver->getVolumeName());

        // NOTE: This causes addDevice(...) to mount the fat partition
        // in /dev/fs/<name> in the global. The VFSDevice will also call
        // back to us to resolve paths, etc.
        VFSDevice* fsDev = new VFSDevice(driver, name);

        // TODO: fsDev is the root directory, so it must impl IFile.


        // Make sure we can navigate both ways
        fsDev->setDriver(driver);
        driver->setDevice(fsDev);

        // Let the world know (this mounts the VFS device into the namespace)
        DeviceManager::addDevice(fsDev);

        FATNode* root = new FATNode(driver, driver->rootCluster);

        // Default "mount name"
        root->setName("root");
        root->size = 0;
        root->isDirectory = root->isSystem = true;
        root->isHidden = false;

        // TODO: maybe place directly under /dev/fs (or let fstab decide) and
        // let the FAT '... ' device node go under the node representing the
        // device on which the FAT is sited (ATA, Floppy, whatever)
        fsDev->addChild(root);

        // Read root directory and extract volume name. This fills this FAT VFS
        // namespace node with root entries, to speed up initial access.
        //driver->scanDirectory(driver->rootCluster, fsDev);
        driver->scanDirectory(driver->rootCluster, root);

        if(driver->readFreeClusters() != E_OK)
            out << "Couldn't read free clusters" << eol;
    }

    log << "VOLUME: " << driver->volumeId << eol;

    return driver;
}

/** @see FileSystem#mount */
error_t FATDriver::mount(Node* parent)
{
    return E_NO_IMPL;
}

/* See declaration */
uint32 FATDriver::getFATEntry(Buffer& fatchunk, int index)
{
    uint8* slot = fatchunk.data() + ((index*facts[type].fatBits)/8);
    uint32 entry = 0;

    switch(type)
    {
        case FAT12:

            // TODO: this is completely untested

            // We access the slot as 16 bits, but only want 12. If the
            // slot is at an odd cluster position, we need the upper
            // 12 bits, else the lower 12 bits.
            entry = Convert::le2host((uint16*)slot);

            if(index & 1)
                entry >>= 4;
            else
                entry &= 0x0FFF;

            break;

        case FAT16:
            entry = Convert::le2host((uint16*)slot);
            break;

        case FAT32:

            // Note the masking, i.e 0x10000000 is in fact a free cluster,
            // because only the lower 28 bits matter.
            entry = Convert::le2host((uint32*)slot) & facts[type].lbaMask;
            break;
    };

    return entry;
}

/* See declaration */
uint32 FATDriver::getFreeCluster()
{
    uint32 res = 0;

    FreeEntry* fe = freeEntries.peek();
    if(fe)
    {
        // Shrink the span
        res = fe->startCluster++;
        fe->count--;

        // Did we exhaust the span?
        if(fe->count == 0)
        {
            delete freeEntries.pop();
        }
    }

    // Zero the cluster; it will be used for stuff like directory clusters
    // where each entry must be zero. After writing, we rewind since we're
    // called from code that doesn't expect the channel position to change.
    fpos_t cur = channel->getPosition();
    if(channel->setPosition(cluster2pos(fe->startCluster)) == E_OK)
    {
        channel->write(nullCluster, bytesPerCluster);
        channel->setPosition(cur);
    }
    else
        res = 0;

    return res;
}

/* See declaration */
error_t FATDriver::readFreeClusters()
{
    error_t res = E_OK;

    foreach_ref(freeEntries)
    {
        delete freeEntries.currentItem();
    }
    freeEntries.clear();

    uint32 fatStart = getFATEntrySector(0) * bytesPerSect ;

    // FAT tables can be huge; we don't want to consume huge amounts of memory,
    // so we read it blockwise.
    // We use blocks of about 32K. You can pick any number, as long as it's a
    // multiple of 12, 16 and 32.
    const int CHUNK_SIZE = 30720;
    int CHUNK_SIZE_ENTRIES = (CHUNK_SIZE*8) / facts[type].fatBits;

    Buffer fatchunk(CHUNK_SIZE);
    channel->setPosition(fatStart);

    ldiv_t chunks = ldiv(sizeFAT * bytesPerSect, fatchunk.capacity());

    printf("   Chunk count is %d\n", (int)chunks.quot);

    // Just for stats
    int avail=0, taken=0;

    // Start-cluster of freespan
    uint32 prevstart = UINT32_MAX;
    uint32 currentClusterEntry = 0;
    FreeEntry* freeEntry = null;

    // Get whole FAT chunks
    for(long i=0; res == E_OK && i < chunks.quot; i++)
    {
        channel->read(fatchunk, fatchunk.capacity(), true);
        res = Thread::getLastError();

        for(int j=0; res == E_OK && j < CHUNK_SIZE_ENTRIES; j++, currentClusterEntry++)
        {
            uint32 entry = getFATEntry(fatchunk, j);

            if(entry == 0)
            {
                avail++;

                // Does this start a span?
                if(prevstart == UINT32_MAX)
                {
                    prevstart = currentClusterEntry;
                    freeEntry = new FreeEntry(prevstart, 1);
                    freeEntries.add(freeEntry);
                }
                else
                    freeEntry->count++;
            }
            else
            {
                taken++;

                // Does this end a span?
                if(prevstart != UINT32_MAX)
                {
                    prevstart = UINT32_MAX;
                    freeEntry = null;
                }
            }
        }
    }

    printf("   Remaining chunk is %d bytes\n", (int)chunks.rem);

    // Get last partial FAT chunk
    channel->read(fatchunk, chunks.rem, true);
    res = Thread::getLastError();

    CHUNK_SIZE_ENTRIES = (chunks.rem*8) / facts[type].fatBits;
    for(int j=0; res == E_OK && j < CHUNK_SIZE_ENTRIES; j++, currentClusterEntry++)
    {
        uint32 entry = getFATEntry(fatchunk, j);

        if(entry == 0)
        {
            avail++;

            // Does this start a span?
            if(prevstart == UINT32_MAX)
            {
                prevstart = currentClusterEntry;
                freeEntry = new FreeEntry(prevstart, 1);
                freeEntries.add(freeEntry);
            }
            else
            {
                freeEntry->count++;
            }
        }
        else
        {
            taken++;

            // Does this end a span?
            if(prevstart != UINT32_MAX)
            {
                // Prepare a new span
                prevstart = UINT32_MAX;
                freeEntry = null;
            }
        }
    }

    // Debug: Dump spans
/*
    avail = 0;
    foreach_ref(freeEntries)
    {
        FreeEntry* fe = freeEntries.currentItem();
        printf("   From cluster %d: %d free clusters\n",
               fe->startCluster, fe->count);

        avail += fe->count;
    }
    printf("%d FREE CLUSTERS, %d NON-FREE\n", avail, taken);
*/

    return res;
}

/* See declaration */
error_t FATDriver::xfer(FATNode* node, fpos_t offset, Buffer& buffer,
                        bool write, bool expand, size_t& xfered)
{
    error_t res = E_OK;
    fpos_t cluster = node->startCluster;
    xfered = 0;

    // TODO: Change the code to work on the buffer directory. However, that
    //       means that the Channel's should work on Buffer's to.
    fpos_t remBytes = buffer.capacity();
    uint8* buf = buffer.data();

    // Upper layer must not attempt to read/write zero bytes or use a
    // bogus buffer
    assert(remBytes > 0 && buf != null);

    // We reset offset later on, so compute the potentially new size here
    uint32 lastOffset = (uint32)offset + buffer.capacity();

    // Number of clusters, starting from startBlock, to ignore
    fpos_t ignore =  offset / bytesPerCluster;
    offset =  offset % bytesPerCluster;

//printf("IGNORING %Ld CLUSTERS, OFFSET: %Ld, CAPACITY: %Ld, BUF: 0x%x\n", ignore, offset, remBytes, buf);

    // We have to follow the chain of clusters until we find the one containing
    // the offset. If necessary, expand the file by allocating new clusters.
    for(fpos_t i=0; res == E_OK && i < ignore; i++)
    {
        fpos_t prevCluster = cluster;
        cluster = getNextCluster(cluster);
        assert(cluster != 0);

        if(cluster != 0 && cluster >= facts[type].eocMark)
        {
            if(write && expand)
            {
                uint32 newCluster = getFreeCluster();
                if(newCluster > 0)
                {
                    // Update prev cluster's next ptr.
                    res = updateNextCluster(prevCluster, newCluster);
                    if(res == E_OK)
                        cluster = newCluster;
                }
                else
                {
                    fprintf(stderr, "No more space left on device");
                    res = E_DEVICE_FULL;
                }
            }
            else
            {
                fprintf(stderr, "EOC cluster %Ld >= %u\n", cluster, facts[type].eocMark);
                res = E_RANGE;
            }
        }
        else if(cluster == facts[type].badClusterMark)
            res = E_DEVICE_ERROR;
    }

//printf("xfer: rembytes = %u from pos %u\n", (uint32)remBytes, cluster2pos((uint32)cluster));

    // Transfer
    while(res == E_OK && remBytes > 0)
    {
        channel->setPosition(cluster2pos((uint32)cluster) + offset);
        uint32 xferSize = Math::min(remBytes, (fpos_t)bytesPerCluster - offset);

        if(write)
        {
            if(channel->write(buf, xferSize, true) < (int)xferSize)
                res = Thread::getLastError();
        }
        else
        {
            if(channel->read(buf, xferSize, true) < (int)xferSize)
                res = Thread::getLastError();
        }

        // Offsetting into a cluster is relevant only the first round
        offset = 0;

//printf("xfering: res=%d, rembytes = %Ld, xferSize=%u\n", res, remBytes, xferSize);

        // Continue with next cluster
        if(res == E_OK && remBytes > 0)
        {
            // Update transfer stats
            xfered += xferSize;

            buf += xferSize;
            fpos_t prevCluster = cluster;
            cluster = getNextCluster(cluster);
            assert(cluster != 0);

            if(cluster != 0 && cluster >= facts[type].eocMark)
            {
                if(write && expand)
                {
                    uint32 newCluster = getFreeCluster();
                    if(newCluster > 0)
                    {
                        // Update prev cluster's next ptr.
                        res = updateNextCluster(prevCluster, newCluster);
                        if(res == E_OK)
                            cluster = newCluster;
                    }
                    else
                        fprintf(stderr, "No free clusters\n");
                }
                else
                {
                    fprintf(stderr, "EOC cluster %Ld >= %u\n", cluster, facts[type].eocMark);
                    res = E_RANGE;
                }
            }
            else if(cluster == facts[type].badClusterMark)
                res = E_DEVICE_ERROR;
        }

        remBytes -= xferSize;

    }// xfer loop

    if(res == E_OK)
    {
        // If writing and the size increased, update directory entry.
        // TODO: must always update DOS TIMESTAMP when writing

        if(write && lastOffset > (size_t)node->getSize())
        {
            if(expand)
            {
                uint8 dirEntry[32];
                assert(channel->setPosition(node->shortEntryOffset) == E_OK);
                assert(channel->read(dirEntry, 32) == 32);

                lastOffset = Convert::host2le(lastOffset);
                memcpy(&dirEntry[FAT_DIRENT_IDX_FILESIZE], &lastOffset, sizeof(uint32));

                assert(channel->setPosition(node->shortEntryOffset) == E_OK);
                assert(channel->write(dirEntry, 32) == 32);

                node->size = lastOffset;
            }
            else
                res = E_RANGE;
        }
    }
    else
    {
        fprintf(stderr, "[FAT] xfer failed: %d\n", res);
        Thread::setLastError(res);
    }

    return res;
}

/** DEBUG: read file contents */
error_t FATDriver::readFile(uint32 cluster, uint32 remainingBytes, uint8* buf)
{
    error_t res = E_OK;

    while(res == E_OK && remainingBytes > 0)
    {
        uint32 lba = cluster2lba(cluster);
        channel->setPosition(lba * bytesPerSect);

        uint32 toRead = Math::min(remainingBytes, bytesPerCluster);
        remainingBytes -= toRead;

        if(channel->read(buf, toRead) < (int)toRead)
            return E_DEVICE_ERROR;

        buf += toRead;

        // Continue with next cluster, if needed
        if(remainingBytes > 0)
        {
            cluster = getNextCluster(cluster);

            if(cluster >= facts[type].eocMark)
            {
                fprintf(stderr, "EOC cluster %Ld >= %u\n", cluster, facts[type].eocMark);
                res = E_RANGE;
            }
            else if(cluster == facts[type].badClusterMark)
                res = E_DEVICE_ERROR;
        }
    }

    return res;
}

/* See declaration */
error_t FATDriver::scanDirectory(uint32 cluster, Node* parentNode,
                                 String* search, FATNode** target)
{
    error_t res = E_OK;

    enum mode {
        MODE_ENUMERATE,
        MODE_SEARCH,
        MODE_ALLOC,
    };

    int mode = MODE_ENUMERATE;

    if(parentNode == null && search == null && target != null && *target != null)
        mode = MODE_ALLOC;
    else if(parentNode != null)
        mode = MODE_ENUMERATE;
    else if(search != null)
        mode = MODE_SEARCH;
    else
        assert(false);

    // There's 32-byte dir entries in all FAT types
    uint8 entry[FAT_DIRENT_SIZE];

    // This is used to keep track of long-name fragments and their checksum
    List<String*> longEntries;
    int chksum = -1;

    bool done = false;

    // This is subtle. If this is true, it means that we are allocating a
    // direntry slot. We found one, but it was the very last in a cluster.
    bool terminateDir = false;

    // Spin through the clusters
    while(!done && res == E_OK)
    {
        if(type == FAT32)
        {
            channel->setPosition(cluster2pos(cluster));

            int dirEntryCount = bytesPerCluster / FAT_DIRENT_SIZE;
            for(int i=0; !done && i < dirEntryCount; i++)
            {
                channel->read(entry, FAT_DIRENT_SIZE);

                // A long-name entry?
                if((entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_LONG_TEXT) == FAT_ATTR_LONG_TEXT &&
                   (entry[0] != FAT_DIRENT_FREE))
                {
                    int newchecksum = (int)entry[FAT_DIRENT_IDX_CHECKSUM];
                    if(chksum != -1 && newchecksum != chksum)
                    {
                        log << "[FAT] Invalid checksum in long entry" << eol;

                        res = E_DEVICE_ERROR;
                        break;
                    }

                    // We'll compare this later on, when we get to the short entry
                    chksum = newchecksum;

                    unichar ch[13];

                    // First 5 unichars
                    int nameIdx = 0;
                    for(int j=1; j<11; j+=2)
                    {
                        ch[nameIdx] = Convert::le2host((uint16*)&entry[j]);
                        nameIdx++;
                    }

                    // Next 6
                    for(int j=14; j < 14+12;  j+=2)
                    {
                        ch[nameIdx] = Convert::le2host((uint16*)&entry[j]);
                        nameIdx++;
                    }

                    // Last 2
                    for(int j=28; j < 32;  j+=2)
                    {
                        ch[nameIdx] = Convert::le2host((uint16*)&entry[j]);
                        nameIdx++;
                    }

                    // Sloooow - fix string to allocate in blocks!
                    String* partialName = new String();
                    for(int j=0; j < 13; j++)
                    {
                        if(ch[j] == 0x00)
                            break;

                        *partialName << ch[j];
                    }

                    longEntries.push(partialName);

                    // move on
                    continue;

                }// long text

                // Entry is free, but there may others following
                if(entry[0] == FAT_DIRENT_FREE)
                {
                    if(mode == MODE_ALLOC)
                    {
                        printf("FAT ALLOC CASE 1\n");

                        // Grab the entry
                        (*target)->shortEntryOffset =
                            (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                        done = true;
                    }

                    continue;
                }
                // Entry is free and is in fact the *last* free entry
                else if(entry[0] == FAT_DIRENT_FREE_END)
                {
                    if(mode == MODE_ALLOC)
                    {
                        if((i+1) < dirEntryCount)
                        {
                            printf("FAT ALLOC CASE 2.0\n");

                            // Grab the entry
                            (*target)->shortEntryOffset =
                                (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                            // Make next entry the last free one
                            channel->write(FAT_DIRENT_FREE_END);
                            channel->setPosition(channel->getPosition()-1);
                        }
                        else
                        {
                            printf("FAT ALLOC CASE 2.1\n");

                            // Grab the entry
                            (*target)->shortEntryOffset =
                                (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                            // The first entry of the next cluster will be set
                            // to FAT_DIRENT_FREE_END
                            terminateDir = true;
                            done = false;

                            break;
                        }
                    }

                    done = true;
                    break;
                }
                else if(entry[0] == FAT_DIRENT_KANJI_LEAD)
                {
                    // Unicode fixup; now, FAT_DIRENT_KANJI_LEAD (0x05) is a
                    // valid ordinal for long file names, so check that first
                    if((entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_LONG_TEXT)
                       != FAT_ATTR_LONG_TEXT)
                    {
                        entry[0] = 0xE5;
                    }
                }

                // Set volume id? Note that no other attr bits can be set.
                if(entry[FAT_DIRENT_IDX_ATTR] == FAT_ATTR_VOLUMEID)
                {
                    memcpy(volumeId, entry, FAT_VOLUMEID_LEN);
                    volumeId[FAT_VOLUMEID_LEN] = 0;
                    continue;
                }

                // If this hits, the above test where *only* the volume id is
                // set failed, and yet we have an entry with a volume id bit
                // (along with other flags). We treat it as orphan.
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_VOLUMEID)
                {
                    continue;
                }

                //
                // When we get to this point, we're at a short directory entry.
                //

                // Handle accumulated long name, if any
                String* longName = new String();
                String* partialName = longEntries.pop();
                while(partialName)
                {
                    *longName << *partialName;
                    delete partialName;

                    // next
                    partialName = longEntries.pop();
                }

#if FAT_DEBUG_LEVEL > 2
                if(out.size() > 0)
                    out << eol;
#endif

                // Short name. Note that we make room for the '.' (which will be
                // added by fixShortName).
                char name[13];
                name[11] =0;
                memcpy(name, entry, 11);

                // Is it an orphan?
                if(chksum != -1 && chksum != checksum((uint8*)name))
                {
                    log << "[FAT] Invalid checksum in long entry (orphan) " << eol;

                    chksum = -1;
                    continue;
                }

                // Prepare for next entry
                chksum = -1;

    #if FAT_DEBUG_LEVEL > 2
                out << name << ", A=" << H(entry[FAT_DIRENT_IDX_ATTR]) << " " << eos;
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_DIRECTORY)
                    out << " D";
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_HIDDEN)
                    out << "H";
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_SYSTEM)
                    out << "S";
    #endif
                uint32 fileSize = Convert::le2host(*(uint32*)&entry[FAT_DIRENT_IDX_FILESIZE]);

    #if FAT_DEBUG_LEVEL > 2
                out << ", size=" << fileSize << " bytes";
                out << eol;

                // Long name
                if(longName->size())
                    out << *longName << eol;
    #endif

                // Cluster
                uint32 startCluster = Convert::le2host((uint16*)&entry[FAT_DIRENT_IDX_CLUSTER_HI]) << 16;
                startCluster |= Convert::le2host((uint16*)&entry[FAT_DIRENT_IDX_CLUSTER_LO]);

                collapseShortName(name);
                FATNode* fnode = null;

                if( mode == MODE_ENUMERATE ||
                   (mode == MODE_SEARCH && search->compare(name) == 0))
                {
                    fnode = new FATNode(this, startCluster);
                    fnode->size = fileSize;
                    fnode->isDirectory = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_DIRECTORY;
                    fnode->isHidden = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_HIDDEN;
                    fnode->isSystem = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_SYSTEM;

                    // Records the volume offset of the directory entry. If the
                    // volume is defragmented, or the file is deleted, all Nodes
                    // referencing this entry must be invalidated (this is a
                    // TODO)
                    fnode->shortEntryOffset = channel->getPosition() - FAT_DIRENT_SIZE;

                    // TODO: Long name should be made available through IFile
                    fnode->setName(name);

                    if(search == null)
                    {
                        parentNode->addChild(fnode);
                    }
                    else
                    {
                        *target = fnode;

                        done = true;
                        break;
                    }
                }

            }// foreach entry
        }
        // TODO: should be able to use the same code for FAT12 and FAT16
        else if(type == FAT16)
        {
            int dirEntryCount = 0;

            // In FAT16, the root dir starts at a fixed area
            if(cluster == 0)
            {
                uint32 dataStartLba = reservedSects +  (numFATs * sizeFAT);
                uint32 lbaRoot = dataStartLba;
                channel->setPosition(lbaRoot * bytesPerSect);

                // Note that rootEntries contains the number of available slots
                // in the root directory, not the actual number of used
                // directory entries.
                dirEntryCount = rootEntries;
            }
            else
            {
                channel->setPosition(cluster2pos(cluster));
                dirEntryCount = bytesPerCluster / FAT_DIRENT_SIZE;
            }

            for(int i=0; !done && i < dirEntryCount; i++)
            {
                if(channel->read(entry, FAT_DIRENT_SIZE) == -1)
                {
                    res = Thread::getLastError();
                    printf("read() failed: %d\n", res);

                    done = true;
                    break;
                }

                // Entry is free, but there may others following
                if(entry[0] == FAT_DIRENT_FREE)
                {
                    if(mode == MODE_ALLOC)
                    {
                        printf("FAT ALLOC CASE 1\n");

                        // Grab the entry
                        (*target)->shortEntryOffset =
                            (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                        done = true;
                    }

                    continue;
                }
                // Entry is free and is in fact the *last* free entry
                else if(entry[0] == FAT_DIRENT_FREE_END)
                {
                    if(mode == MODE_ALLOC)
                    {
                        if((i+1) < dirEntryCount)
                        {
                            printf("FAT ALLOC CASE 2.0\n");

                            // Grab the entry
                            (*target)->shortEntryOffset =
                                (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                            // Make next entry the last free one
                            channel->write(FAT_DIRENT_FREE_END);
                            channel->setPosition(channel->getPosition()-1);
                        }
                        else
                        {
                            printf("FAT ALLOC CASE 2.1\n");

                            // Grab the entry
                            (*target)->shortEntryOffset =
                                (uint32)channel->getPosition() - FAT_DIRENT_SIZE;

                            // The first entry of the next cluster will be set
                            // to FAT_DIRENT_FREE_END
                            terminateDir = true;
                            done = false;

                            break;
                        }
                    }

                    done = true;
                    break;
                }

                // Set volume id? Note that no other bits can be set.
                if(entry[FAT_DIRENT_IDX_ATTR] == FAT_ATTR_VOLUMEID)
                {
                    memcpy(volumeId, entry, FAT_VOLUMEID_LEN);
                    volumeId[FAT_VOLUMEID_LEN] = 0;
                    continue;
                }

                // If this hits, the above test where *only* the volume id is
                // set failed, and yet we have an entry with a volume id bit
                // (along with other flags). We treat it as orphan (can this
                // even happened on fat12/16?)
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_VOLUMEID)
                    continue;

                // Short name. Note that we make room for the '.' (which will be
                // added by fixShortName).
                char name[13];
                name[11] =0;
                memcpy(name, entry, 11);

    #if FAT_DEBUG_LEVEL > 2
                out << name << ", A=" << H(entry[FAT_DIRENT_IDX_ATTR]) << " " << eos;
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_DIRECTORY)
                    out << " D";
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_HIDDEN)
                    out << "H";
                if(entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_SYSTEM)
                    out << "S";
    #endif
                uint32 fileSize = Convert::le2host(*(uint32*)&entry[FAT_DIRENT_IDX_FILESIZE]);

    #if FAT_DEBUG_LEVEL > 2
                out << ", size=" << fileSize << " bytes";
                out << eol;
    #endif

                // Cluster; in fat12/16, only the lower 16 bits are used
                uint32 startCluster = Convert::le2host((uint16*)&entry[FAT_DIRENT_IDX_CLUSTER_LO]);

                collapseShortName(name);
                FATNode* fnode = null;

                if( mode == MODE_ENUMERATE ||
                   (mode == MODE_SEARCH && search->compare(name) == 0))
                {
                    fnode = new FATNode(this, startCluster);
                    fnode->size = fileSize;
                    fnode->isDirectory = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_DIRECTORY;
                    fnode->isHidden = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_HIDDEN;
                    fnode->isSystem = entry[FAT_DIRENT_IDX_ATTR] & FAT_ATTR_SYSTEM;

                    // Records the position of the directory entry. If the volume
                    // is defragmented, or the file is deleted, all Nodes referencing
                    // this entry must be invalidated (this is a TODO)
                    fnode->shortEntryOffset = channel->getPosition() - FAT_DIRENT_SIZE;

                    // TODO: Long name should be made available through IFile
                    fnode->setName(name);

                    if(search == null)
                    {
                        parentNode->addChild(fnode);
                    }
                    else
                    {
                        *target = fnode;

                        done = true;
                        break;
                    }
                }

    #if FAT_DEBUG_LEVEL > 1

                if(strcmp(name, "USPACE1") == 0)
                {
                    loadUserland(startCluster, fileSize);
                }
    #endif
            }// foreach entry
        }
        else if(type == FAT12)
        {
            log << "FAT12 not supported yet" << eol;
            res = E_INVALID;
        }

        if(!done)
        {
            // Continue with next cluster, if any
            uint32 prevCluster = cluster;
            cluster = getNextCluster(cluster);

            // Important detail: the EOC is >= eocMark, not necessarily equal
            if(cluster >= facts[type].eocMark)
            {
    #if FAT_DEBUG_LEVEL > 2
                log << "EOC mark encountered: " << H(cluster) << eol;
    #endif

                if(mode == MODE_ALLOC)
                {
                    printf("FAT ALLOC CASE 3\n");

                    // We are looking for a free entry, but ran out of clusters. So
                    // we try to extend the chain and grab the first entry.
                    uint32 newCluster = getFreeCluster();
                    if(newCluster > 0)
                        res = updateNextCluster(prevCluster, newCluster);

                    // The loop will now continue as if nothing happened
                    if(res == E_OK)
                        cluster = newCluster;

                    // Maybe we are almost done? This case happends when we
                    // have allocated the very last direntry slot in a cluster,
                    // and that entry was marked as the last-free-dir. What we
                    // need to do is just mark off the first slot in the new
                    // cluster as last-free-dir.
                    if(terminateDir)
                    {
                        res = channel->setPosition(cluster2pos(newCluster));
                        if(res == E_OK)
                        {
                            printf("TERMINATING FAT DIR CHAIN\n");
                            channel->write(FAT_DIRENT_FREE_END);
                        }

                        done = true;
                    }
                }
                else
                {
                    done = true;
                }
            }
            else if(cluster == facts[type].badClusterMark)
            {
                log << "[FAT] Bad cluster" << eol;
                res = E_DEVICE_ERROR;
            }
        }

    }// while

    return res;
}

/* See declaration */
error_t FATDriver::remove(FATNode* node)
{
    error_t res = E_INVALID;

    if(node)
    {
        if(node->getIsDirectory())
        {
            NO_IMPL();
        }
        else
        {
            res = channel->setPosition(node->shortEntryOffset);
            if(res == E_OK)
            {
                // Mark as free. Note that FAT_DIRENT_FREE is written when files
                // are created, not when removing.
                channel->write(FAT_DIRENT_FREE);
            }

        }
    }

    return res;
}

/** See declaration */
error_t FATDriver::rename(FATNode* node, const String& newName)
{
    error_t res = E_INVALID;
    if(node)
    {
        res = channel->setPosition(node->shortEntryOffset);
        if(res == E_OK)
        {
            // Write new, exploded, name to the dir entry
            uint8 exploded[11];
            explode(newName, exploded);

            channel->write(exploded, 11);
        }
    }

    return res;
}

/* See declaration */
error_t FATDriver::create(FATNode* parent, const String& name,
                          uint64 attr, FATNode*& created, bool directory)
{
    error_t res = E_OK;

    if(!parent->getIsDirectory())
        res = E_INVALID;
    else
    {
        uint32 newCluster = getFreeCluster();
        if(newCluster > 0)
        {
            printf("Creating file '%S' at %u, in directory %u:%u\n",
                   &name, newCluster, parent->startCluster, parent->shortEntryOffset);

            created = new FATNode(this, newCluster);

            // Abuse scanDirectory to find a free entry
            res = scanDirectory(parent->startCluster, null, null, &created);
            if(res == E_OK)
            {
                // Update directory entry
                res = channel->setPosition(created->shortEntryOffset);

                char dirent[FAT_DIRENT_SIZE];
                memset(dirent, 0, FAT_DIRENT_SIZE);

                // Create directory?
                if(directory)
                    dirent[FAT_DIRENT_IDX_ATTR] = FAT_ATTR_DIRECTORY;

                // TODO: Verify that the name only contains short-name
                // characters (we don't support long names yet)

                explode(name, (uint8*)dirent);
                //name.toAscii(dirent);

                *((uint16*)&dirent[FAT_DIRENT_IDX_CLUSTER_LO]) =
                    (uint16)Convert::host2le(newCluster & 0xFFFF);

                *((uint16*)&dirent[FAT_DIRENT_IDX_CLUSTER_HI]) =
                    (uint16)Convert::host2le((newCluster >> 16) & 0xFFFF);

                // Update the directory entry
                channel->write((const uint8*)dirent, FAT_DIRENT_SIZE);

                // The cluster is no longer free; it's zero length, so EOC the
                // chain.
                if(res == E_OK)
                    res = updateNextCluster(newCluster, facts[type].eocMark);
            }
            else
            {
                printf("scanDirectory failed\n");
            }
        }
        else
        {
            res = E_DEVICE_FULL;
        }
    }

    return res;
}

/* See declaration */
error_t FATDriver::setLength(FATNode* node, size_t newsize)
{
    error_t res = E_INVALID;

    if(node)
    {
        res = channel->setPosition(node->shortEntryOffset +
                                   FAT_DIRENT_IDX_FILESIZE);

        if(res == E_OK)
        {
            // Update dir entry
            uint32 len = Convert::host2le((uint32)newsize);
            channel->write((uint8*)&len, sizeof(uint32));

            // We must update the Node's length before calling xfer, to
            // prevent size entry from beeing written.
            node->size = newsize;

            printf("setLength %u bytes (%u)\n", newsize, len);

            // Figure out how many cluster before and after
            uldiv_t chunks = uldiv(node->size, bytesPerCluster);
            uint32 oldcc = (uint32)chunks.quot;
            oldcc += chunks.rem > 0 ? 1 : 0;

            chunks = uldiv(newsize, bytesPerCluster);
            uint32 newcc = (uint32)chunks.quot;
            newcc += chunks.rem > 0 ? 1 : 0;

            // Truncate?
            if(newcc < oldcc)
            {
                // Spin to the (new) last cluster. If the new size is zero, then
                // we're already there.
                uint32 cluster = node->startCluster;
                uint32 i=1;
                for(; res == E_OK && i < newcc; i++)
                {
                    cluster = getNextCluster(cluster);

                    if(cluster == 0)
                        res = E_DEVICE_ERROR;
                }

                if(res == E_OK)
                {
                    uint32 end = cluster;
                    cluster = getNextCluster(cluster);

                    // End the chain
                    res = updateNextCluster(end, facts[type].eocMark);

                    // Now, mark the remaining clusters as free
                    for(; res == E_OK && i < oldcc; i++)
                    {
                        uint32 freeCluster = cluster;
                        cluster = getNextCluster(cluster);

                        if(freeCluster == 0)
                            res = E_DEVICE_ERROR;
                        else
                            res = updateNextCluster(freeCluster, 0);
                    }
                }
            }
            // Expand? In this case we allocate zero-filled clusters
            else if(newcc > oldcc)
            {
                // We just reuse xfer's logic to expand the file by writing a 0
                // at the very end.
                Buffer zerobuf(1);
                zerobuf.fill(0);

                size_t xfered;
                res = xfer(node, newsize-1, zerobuf, true, true, xfered);
            }
        }
    }

    return res;
}

/** Given a cluster, compute and return lba */
uint32 FATDriver::cluster2lba(uint32 cluster)
{
    uint32 dataStartLba = reservedSects +  (numFATs * sizeFAT);
    uint32 lba = dataStartLba + ((cluster-2) * sectPerCluster);

    if(type == FAT16 || type == FAT12)
    {
        // Move beyond the directory area
        lba+=(rootEntries*32 / bytesPerSect);
    }

    return lba;
}

/* See declaration */
uint32 FATDriver::cluster2pos(uint32 cluster)
{
    // Computes and returns the byte position within the data area, i.e what you
    // would pass on to channel->setPosition(...)
    return cluster2lba(cluster) * bytesPerSect;
}

/* See declaration */
error_t FATDriver::updateNextCluster(uint32 cluster, uint32 nextCluster)
{
    error_t res = E_OK;

    uint32 fatEntryPos = getFATEntrySector(cluster) * bytesPerSect
        + getFATEntryOffset(cluster);
    res = channel->setPosition(fatEntryPos);

    // TODO: support FAT12
    int fatEntrySize = facts[type].fatBits / 8;
    uint8 fatEntry[fatEntrySize];

    if(res == E_OK && type == FAT32)
    {
        nextCluster = Convert::host2le(nextCluster);
        if(channel->write((uint8*)&nextCluster, fatEntrySize) < E_OK)
            res = Thread::getLastError();
    }
    else if(res == E_OK && type == FAT16)
    {
        uint16 next16 = Convert::host2le((uint16)nextCluster);
        if(channel->write((uint8*)&next16, fatEntrySize) < E_OK)
            res = Thread::getLastError();
    }
    else if(res == E_OK && type == FAT12)
    {
        NO_IMPL();
    }

    // End the chain
    if(res == E_OK && nextCluster < facts[type].eocMark)
        res = updateNextCluster(nextCluster, facts[type].eocMark);

    return res;
}

/**
 * Get next cluster index from the FAT. This function may return the EOC
 * value for the current fat type.
 *
 * @param cluster Current cluster index, 0 on error
 */
uint32 FATDriver::getNextCluster(uint32 cluster)
{
    error_t res = E_OK;

    uint32 fatEntryPos = getFATEntrySector(cluster) * bytesPerSect
        + getFATEntryOffset(cluster);

    res = channel->setPosition(fatEntryPos);

    // TODO: support FAT12
    int fatEntrySize = facts[type].fatBits / 8;
    uint8 fatEntry[fatEntrySize];

    if(res == E_OK)
    {
        if(channel->read(fatEntry, fatEntrySize) < E_OK)
            res = Thread::getLastError();
    }

    uint32 nextCluster = 0;

    if(res == E_OK && type == FAT32)
    {
        nextCluster = Convert::le2host((uint32*)&fatEntry[0]) & facts[type].lbaMask;
    }
    else if(res == E_OK && type == FAT16)
    {
        nextCluster = Convert::le2host((uint16*)&fatEntry[0]) & facts[type].lbaMask;
    }
    else if(res == E_OK && type == FAT12)
    {
        NO_IMPL();
    }

    if(res != E_OK)
        printf("ERROR in getNextCluster: %d\n", res);

    return res == E_OK ? nextCluster : 0;
}

/** Sector on which the FAT entry for the given cluster resides */
uint32 FATDriver::getFATEntrySector(uint32 cluster)
{
    return reservedSects + (getFATEntry(cluster) / bytesPerSect);
}

/** Offset on sector in which the FAT entry for the given cluster resides */
uint32 FATDriver::getFATEntryOffset(uint32 cluster)
{
    return getFATEntry(cluster) % bytesPerSect;
}

/** Given a cluster index, get the corresponding entry index in the FAT */
uint32 FATDriver::getFATEntry(uint32 cluster)
{
    if(type == FAT32)
    {
        return cluster * 4;
    }
    else if(type == FAT16)
    {
        return cluster * 2;
    }
    else
    {
        // TODO:
        NO_IMPL();
        return 0;
    }
}


// TODO: just testing loading ELF files
void FATDriver::loadUserland(uint32 startCluster, uint32 fileSize)
{
    uint64 oldpos = channel->getPosition();

    uint8* buf = new uint8[fileSize];
    readFile(startCluster, fileSize, buf);
    Kernel::instance().userbuf = buf;

    // restore pos
    channel->setPosition(oldpos);
}
