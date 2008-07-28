/**
 * The Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * The PATA bus consists of two channels, each having 0..2 drives.
 *
 * Detecting ATA devices is not standardized and a lot of surprisingly complex and
 * mysterious detection code exists, mostly to handle very old drives and controllers.
 *
 * The way I've implemented device probing is very simple - just attempt to (p)identify
 * and if an interrupt occurs within a timeout period without an ABORT error, then I
 * assume there's a drive there.
 *
 * The code probes PCI IDE controllers, but doesn't use it for anything beyond reporting
 * its presence.
 */

#include <stdio.h>
#include <kernel/drivers/ata/ATABus.h>
#include <kernel/drivers/ata/ATADriver.h>
#include <kernel/drivers/fs/generic/Partitions.h>
#include <kernel/drivers/bus/pci/PCIDatabase.h>
#include <arch/drivers/bus/pci/PCIConfig.h>
#include <arch/Delay.h>

ATABus* ATABus::instance = null;

/**
 * C'tor
 */
ATABus::ATABus()
{
}

/**
 * Initializes the ATA subsystem
 */
ATABus* ATABus::getInstance()
{
    if(ATABus::instance == null)
    {
        ATABus::instance = new ATABus();
    }

    return ATABus::instance;
}

/** Initialize ATA subsystem */
ATABus* ATABus::initialize()
{
    ATABus* bus = ATABus::getInstance();
    bus->probe();

    return bus;
}

/**
 * Find PCI IDE controllers and ATA devices
 */
void ATABus::probe()
{
    DriverPCI* pci = DriverPCI::getInstance();

    // Find PCI IDE controllers. Note that there can be more than one, such as a
    // PIIX4 controller and a SATA RAID controller.
    // TODO: move this lookup process to, uhm, the PCI driver
    enumPCIDevices(pci->rootBus);

    foreach(&pci->rootBus->children)
    {
        enumPCIDevices(pci->rootBus->children.currentItem());
    }

    // Find ATA devices and install IRQ handler, but only if we found a PCI IDE controller.
    if(ideControllers.count() > 0)
    {
        enumATADevices();
    }

}

/** Collect and report PCI-IDE controllers */
void ATABus::enumPCIDevices(PCIBus* bus)
{
    foreach(&bus->devices)
    {
        PCIDevice* dev = bus->devices.currentItem();
        kassert(dev != null);
        kassert(dev->classInfo != null);

        if(dev->classInfo->BaseClass == PCI_CLASS_MASS_STORAGE_CONTROLLER &&
           dev->classInfo->SubClass == PCI_SUBCLASS_IDE)
        {
            out << "[ATA] Found PCI-IDE controller: " << dev->deviceInfo->ChipDesc
                << ", Prog IF: " << dev->classInfo->ProgIf;

            if(dev->classInfo->ProgIf & 0x80)
            {
                out << " (Bus Mastering OK)";
            }

            out << eol;

            ideControllers.add(dev);

            uint32 regbase;
            PCIConfig::read(dev->getBus()->busNum, dev->deviceNum,
                            dev->functionNum, 0x20/4, 4, &regbase);
            // Base address in 15:4
            regbase = regbase & 0x0000FFF0;

            printf("IDE PIIX IO BASE: 0x%x\n", regbase);

            uint8 status = in8(regbase + 2);
            printf("IDE PCI Status Register: 0x%x\n", status);

        }
    }
}

/** Find ATA(PI) devices */
void ATABus::enumATADevices()
{
    ATAChannel* c1 = new ATAChannel(this, ATA_IOBASE_CONTROLLER_1, 14);

    ATADriver* d1 = ATADriver::initialize(c1,0);
    if(d1 && !d1->getIsPacket())
    {
        out << "Self testing..." << eol;
        d1->test();

        // Figure out partitions and file systems
        PartitionManager pm((Channel*)d1->getInterface(IIDChannel));
    }

    ATADriver* d2 = ATADriver::initialize(c1,1);
    if(d2 && !d2->getIsPacket())
    {
        out << "Self testing..." << eol;
        d2->test();

        // Figure out partitions and file systems
        PartitionManager pm((Channel*)d2->getInterface(IIDChannel));
   }

    ATAChannel* c2 = new ATAChannel(this, ATA_IOBASE_CONTROLLER_2, 15);

    ATADriver* d3 = ATADriver::initialize(c2,0);
    if(d3 && !d3->getIsPacket())
    {
        out << "Self testing..." << eol;
        d3->test();

        // Figure out partitions and file systems
        PartitionManager pm((Channel*)d3->getInterface(IIDChannel));
    }

    ATADriver* d4 = ATADriver::initialize(c2,1);
    if(d4 && !d4->getIsPacket())
    {
        out << "Self testing..." << eol;
        d4->test();

        // Figure out partitions and file systems
        PartitionManager pm((Channel*)d4->getInterface(IIDChannel));
    }

}
