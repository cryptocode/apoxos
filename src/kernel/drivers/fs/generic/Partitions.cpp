/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 */

#include <kernel/drivers/fs/generic/Partitions.h>
#include <kernel/drivers/fs/fat/FAT.h>
#include <kernel/drivers/DeviceManager.h>

namespace
{
    /* File system classes; this is used to pick the right filesystem driver */
    #define FSCLASS_NONE            0
    #define FSCLASS_UNSUPPORTED     1
    #define FSCLASS_APOX            2
    #define FSCLASS_FAT             3

    /** Identifies a partition */
    struct PartitionType
    {
        int id;
        char* name;
        int fsclass;
    };

    /**
     * A few partition types of interest; primary source is http://www.win.tue.nl/~aeb/partitions/partition_types-1.html
     *
     * For most of the time, we only use it for reporting
     */
    PartitionType partitionTypes[] =
    {
        {0x00, "Empty", FSCLASS_NONE},
        {0x01, "FAT-12", FSCLASS_FAT},
        {0x04, "FAT-16", FSCLASS_FAT},
        {0x05, "DOS 3.3+ Extended", FSCLASS_FAT},
        {0x06, "DOS 3.31+ 16-bit FAT", FSCLASS_FAT},
        {0x07, "NTFS", FSCLASS_UNSUPPORTED},
        {0x0B, "Win95 OSR FAT32", FSCLASS_FAT},
        {0x0C, "Win95 OSR FAT32 LBA-mapped", FSCLASS_FAT},
        {0x0E, "Win95 DOS 16-bit LBA-mapped", FSCLASS_FAT},
        {0x0F, "Win95 Extended LBA-mapped", FSCLASS_FAT},
        {0x2A, "Syllable/AtheOS", FSCLASS_UNSUPPORTED},
        {0x2B, "SyllableSecure", FSCLASS_UNSUPPORTED},
        {0x39, "Plan9", FSCLASS_UNSUPPORTED},
        {0x3C, "PowerQuest Recovery", FSCLASS_UNSUPPORTED},
        {0x3D, "Hidden Netware Partition", FSCLASS_UNSUPPORTED},
        {0x41, "Linux/Minix", FSCLASS_UNSUPPORTED},
        {0x42, "Windows 2000 propriatary partition", FSCLASS_UNSUPPORTED},
        {0x4D, "QNX", FSCLASS_UNSUPPORTED},
        {0x52, "CP/M", FSCLASS_UNSUPPORTED},
        {0x82, "Linux Swap", FSCLASS_UNSUPPORTED},   // *can* be solaris x86
        {0x83, "Linux", FSCLASS_UNSUPPORTED},
        {0x85, "Linux Extended", FSCLASS_UNSUPPORTED},
        {0x9C, "Apox", FSCLASS_APOX},         // yup, that's us
        {0xA6, "OpenBSD", FSCLASS_UNSUPPORTED},
        {0xA7, "NeXTStep", FSCLASS_UNSUPPORTED},
        {0xA8, "Mac OS-X", FSCLASS_UNSUPPORTED},
        {0xA9, "NetBSD", FSCLASS_UNSUPPORTED},
        {0xBF, "Solaris x86", FSCLASS_UNSUPPORTED},  // 0x82 conflicted with linux swap
        {0xD8, "CP/M-86", FSCLASS_UNSUPPORTED},
        {0xEB, "BeOS", FSCLASS_UNSUPPORTED},
        {0xEC, "SkyOS SkyFS", FSCLASS_UNSUPPORTED},
        {0xFA, "Bochs", FSCLASS_UNSUPPORTED},
        {0xFB, "VMWare", FSCLASS_UNSUPPORTED},
        {0xFC, "VMWare Swap", FSCLASS_UNSUPPORTED},
        {0xFD, "Linux RAID partition", FSCLASS_UNSUPPORTED},
    };
}

/**
 * Look up partition type descriptor based on it's (semi)official type id
 */
PartitionType* getPartitionType(int typeId)
{
    int typeCount = sizeof(partitionTypes) / sizeof(PartitionType);

    for(int i=0; i < typeCount ; i++)
    {
        if(partitionTypes[i].id == typeId)
        {
            return &partitionTypes[i];
        }
    }

    return null;
}

/**
 * Creates a partition manager using the provided channel (which usually
 * represents an entire disk)
 */
PartitionManager::PartitionManager(Channel* ch)
{
    kassert(ch != null);

    uint8* boot = new uint8[512];

    // First, read the device's boot sector
    ch->setPosition(0);
    int read = ch->read(boot, 512);

    if(read == 512)
    {
        // Parse partition table and create FS drivers as we encounter supported
        // file systems.
        if(boot[510] == 0x55 && boot[511] == 0xAA)
        {
            out << "MBR DETECTED" << eol;

            uint8* pt = &boot[446];

            for(int i=0; i < 4; i++)
            {
                // It's little endian
                uint64 start = pt[8]  | (pt[9] << 8)  | (pt[10] << 16) | (pt[11]<<24);
                uint64 sectorCount =  pt[12] | (pt[13] << 8) | (pt[14] << 16) | (pt[15]<<24);

                PartitionType* partitionType = getPartitionType(pt[4]);

                if(partitionType != null && partitionType->fsclass != FSCLASS_NONE)
                {
                    out << "Partion table " << i<< " has type " << H(pt[4]) << " " << partitionType->name
                        << ", start: " << start
                        << ", size: " << ((sectorCount*512/*assuming sector size here*/)/1000/1000) << " MB" << eol;

                    // Create filessystem driver and a device instance (todo) if we support this FS class
                    if(partitionType->fsclass == FSCLASS_FAT)
                    {
                        out << "Creating FAT driver" << eol;

                        PartitionChannel* pc =
                            new PartitionChannel(ch, start*512, sectorCount*512);
                        kassert(pc != null);

                        FATDriver* fsDriver = FATDriver::initialize(pc);
                    }
                    else if(partitionType->fsclass == FSCLASS_APOX)
                    {
                        out << "Creating APOX object store driver" << eol;

                        // bah
                        NO_IMPL();
                    }
                }
                else
                {
                    if(partitionType != null)
                        out << "Unsupported or empty partition, id: " << partitionType->id << eol;
                    else
                        out << "Unknown partition type: " << pt[4] << eol;
                }

                // Next partition table entry
                pt+=16;
            }
        }
        else
        {
            out << "MBR NOT DETECTED" << eol;
        }

    }
    else
    {
        out << "Could not read the boot sector" << read << eol;
    }

    delete boot;
}
