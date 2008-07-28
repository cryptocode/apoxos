/**
 * Apox Operating System
 * Copyright 2006-2007, cryptocode
 *
 * PCI Bus Driver implementation. This should comply to PCI Conventional.
 */

#include <kernel/drivers/bus/pci/DriverPCI.h>
#include <kernel/drivers/bus/pci/PCIDatabase.h>
#include <kernel/io/Ports.h>
#include <kernel/Errors.h>
#include <kernel/io/Channel.h>
#include <kernel/Kernel.h>
#include <kernel/Node.h>

#include <arch/drivers/bus/pci/PCIConfig.h>


// FIXME: driver instantiations are hardcoded
#include <kernel/drivers/nic/ne2k/NE2K.h>
#include <kernel/drivers/nic/realtek/RTL8139D.h>
#include <kernel/drivers/nic/amd/PCNet32.h>
#include <kernel/drivers/video/vmware/VMWareDriver.h>

/* PCI Routing  */
#define PCI_PIR_SIGNATURE       "$PIR"
#define PCI_PIR_SIGNATURE_LEN   4

/** The one and only driver instance */
DriverPCI* DriverPCI::instance = null;

/*
 * PCI threats each function in a multifunction devices as separate devices
 *
 * Bits 0:2 - Function
 *      3:7 - Device/slot
 */
#define PCI_DEVICE(dev)     (Bits::getRange(dev, 3, 7))
#define PCI_FUNCTION(dev)   (Bits::getRange(dev, 0, 2))

#define PCI_MAX_DEVICES_PER_BUS     32
#define PCI_MAX_FUNCTIONS_PER_DEV   8

/**
 * Link value / IRQ Bitmap pair for an interrupt pin in a slot
 */
struct PCILinkMap
{
    uint8_t  pin;
    uint16_t possible_irq;

} __attribute__((packed));

/**
 * Slot entry
 */
struct PCIPirSlot
{
    uint8_t  bus;
    uint8_t  devfunc;
    struct PCILinkMap linkmap[4];
    uint8_t  slot;
    uint8_t  reserved;

} __attribute__((packed));

/**
 * PCI Interrupt Routing Table header
 */
struct PCIPirHeader
{
    uint8_t  signature[PCI_PIR_SIGNATURE_LEN];
    uint16_t version;
    uint16_t tableSize;
    uint8_t  router_bus;
    uint8_t  router_devfunc;
    uint16_t exclusive_irq;
    uint32_t compat_vend;
    uint32_t miniport;
    uint8_t  rsvd[11];
    uint8_t  checksum;

} __attribute__((packed));

struct PCIPirTable
{
    struct PCIPirHeader header;
    struct PCIPirSlot slot[1];

} __attribute__((packed));

/**
 * Find PCI Interrupt Routing Table (PIR)
 */
static struct PCIPirTable* pciFindPirTable(void)
{
    // Search for PIR table from 0xF0000 to 0xFFFFF
    uint32_t pirBase = (uint32_t) 0xF0000;
    int range = 0x10000;

    for (; range > 0; range -= 16, pirBase += 16)
    {
        if (memcmp((void*)pirBase, PCI_PIR_SIGNATURE, PCI_PIR_SIGNATURE_LEN) == 0)
        {
            uint16_t size = ((struct PCIPirHeader*)pirBase)->tableSize;

            // The table size must be at least the size of the header, and at
            // most 32 times that.
            if ((size >= (sizeof(struct PCIPirHeader))) &&
                (size <= (sizeof(struct PCIPirHeader) + sizeof(struct PCIPirSlot) * 32)))
            {
                uint8_t checksum = 0;

                for (uint16_t i = 0; i < size; i++)
                {
                    checksum += ((uint8_t*)pirBase)[i];
                }

                if (checksum == 0)
                {
                    return (struct PCIPirTable*)pirBase;
                }
            }
        }
    }

    return null;
}

/**
 * Display the PIR table
 *
 * @note Courtesy of NewOS
 */
static void pciDumpTable(struct PCIPirTable *tbl)
{
    int i, j;
    int entries = (tbl->header.tableSize - sizeof(struct PCIPirHeader)) / sizeof(struct PCIPirSlot);

    int lines = 0;
    for (i = 0; i < entries; i++)
    {
        out << "PIR slot :" << tbl->slot[i].slot << ", bus " <<
            tbl->slot[i].bus << ", device "  <<
            PCI_DEVICE(tbl->slot[i].devfunc) << eol;

        lines++;

        for (j = 0; j < 4; j++)
        {
            lines++;
            out << "    INT" << ('A'+j) << ": pin " << tbl->slot[i].linkmap[j].pin
                << ", possible irq: " << H(tbl->slot[i].linkmap[j].possible_irq) << eol;
        }
    }

}

/** Private c'tor */
DriverPCI::DriverPCI()
{
    probe();
}

/** Initialize driver */
DriverPCI* DriverPCI::initialize()
{
    return getInstance();
}

/** Create singleton */
DriverPCI* DriverPCI::getInstance()
{
    if(instance == null)
    {
        instance = new DriverPCI();
    }

    return instance;
}

/**
 * Probe for PCI busses and devices
 */
void DriverPCI::probe()
{
    PCIConfig::initialize();

    // Validate that configuration method #1 works on this system
    if(PCIConfig::valid())
    {
        out << "[PCI] CONFIG METHOD #1 OK\n";
    }

    pir = pciFindPirTable();

    if(pir)
    {
        out << "[PCI] PIR TABLE FOUND: Signature: "
            << (char)pir->header.signature[0] << (char)pir->header.signature[1]
            << (char)pir->header.signature[2] << (char)pir->header.signature[3]
            << ", Version: " << pir->header.version
            << ", Table size: " << pir->header.tableSize
            << eol;

        out << "[PCI] IRQ Router: ";

        PCIVendorInfo* ven =
            PCIDatabase::getVendorById(LOW16(pir->header.compat_vend));

        if(ven)
            out << ven->VenFull << ", ";
        else
            out << "Unknown vendor, ";

        PCIDeviceInfo* dev =
            PCIDatabase::getDeviceById(LOW16(pir->header.compat_vend),
                                       HIGH16(pir->header.compat_vend));

        if(dev)
            out << dev->ChipDesc;
        else
            out << "Unknown device";

        // Dump PIR
        out << eol;
        pciDumpTable(pir);
    }
    else
    {
        out << "[PCI] PIR TABLE NOT FOUND";
    }

    out.println();

    // Start probing the PCI bus hierachy
    rootBus = new PCIBus(0);
    enumerateBus(rootBus);
}

/**
 * Enumerate the PCI bus hierachy. Calls itself recursively for child buses.
 */
void DriverPCI::enumerateBus(PCIBus* bus)
{
    uint32 val = 0;

    out << "[PCI] Discovering devices on bus "<< bus->busNum << "..." << eol;

    for(int dev=0; dev < PCI_MAX_DEVICES_PER_BUS; dev++)
    {
        uint32 prev_dev=0;

        for(int function=0; function < PCI_MAX_FUNCTIONS_PER_DEV; function++)
        {
            // Read vendo/device config dword
            PCIConfig::read(bus->busNum, dev, function, PCI_CONFIG_VENDOR_DEVICE, 4, &val);
            uint16 vendor = LOW16(val);
            uint16 devid = HIGH16(val);

            // The same device may pop up more than once
            if(devid == prev_dev)
                continue;

            prev_dev = devid;

            // Non-existent devices are identified as vendor 0xFFFF by the host/PCI bridge
            if(vendor != 0xFFFF && vendor != 0 /* VMWare */)
            {
                PCIDeviceInfo* devInfo = PCIDatabase::getDeviceById(vendor, devid);
                PCIVendorInfo* vendorInfo = PCIDatabase::getVendorById(vendor);

                // Make bogus entries
                if(devInfo == null)
                {
                    devInfo = new PCIDeviceInfo();
                    devInfo->VenId = vendor;
                    devInfo->DevId = 0;
                    devInfo->Chip = "Unknown";
                    devInfo->ChipDesc = "Unknown";
                }

                if(vendorInfo == null)
                {
                    vendorInfo = new PCIVendorInfo();
                    vendorInfo->VenId = vendor;
                    vendorInfo->VenShort = "Unknown";
                    vendorInfo->VenFull = "Unknown";
                }

                if(devInfo != null && vendorInfo != null)
                {
                    out << "[PCI] " << devInfo->ChipDesc << " (" << vendorInfo->VenShort << ")" << eol;
                }
                else
                {
                    out << "[PCI] Unknown device found" << eol;
                }

                // IRQ line and pin
                uint32 line=0, pin=0;
                PCIConfig::read(bus->busNum, dev, function, 0x3c, 1, &line);
                PCIConfig::read(bus->busNum, dev, function, 0x3d, 1, &pin);

                // Get header type
                PCIConfig::read(bus->busNum, dev, function, 0x0E /*header type byte offset */, 1, &val);

                out << "   " << ((val & 0x80) ? "Multi" : "Single") << " function device (function "
                    << function << "). Header type " << (val & 0x7F) << eol;

                if(pin > 0)
                    out << "   Interrupt pin " << (char)('A' + pin-1) << " routed to line " << line <<  eol;
                else
                    out << "   Doesn't use interrupts" << eol;

                // pci-pci bridge? TODO: must update these config regs (primary, secondary, subord)
                if(val == 1)
                {
                    //PCIConfig::read(this, bus->busNum, dev, function, 27 /* subordinate bus num*/, 1, &val);
                    //kprintf(", subord: %d", val);
                }

                //
                // Get class, subclass and prog interface info
                // -----------------------------------------
                // | class code | subclass code | prog i/f |
                // -----------------------------------------
                // 23         16 15            8 7         0
                //
                PCIConfig::read(bus->busNum, dev, function,
                                PCI_CONFIG_DWORD_CLASS_REVISION, 4, &val);

                int classcode = val >> 8;
                int revision = val & 0xFF;

                int progif = classcode & 0xFF;
                int subclass = classcode >> 8 & 0xFF;
                int mainclass = classcode >> 16 & 0xFF;

                PCIClassInfo* ci = PCIDatabase::getClassInfo(val);
                if(ci == 0)
                {
                    // Fall back to checking without function prog I/F
                    ci = PCIDatabase::getClassInfo(mainclass, subclass);
                }

                if(ci != 0)
                {
                    // Update program interface, since the PCI database doesn't include bits'n'pieces
                    // like the Bus Mastering bit for PCI IDE controllers.
                    ci->ProgIf = (uint8)progif;

                    out << "   Class code: "   << H(classcode) << ", "
                        << ci->BaseDesc <<":"  << ci->SubDesc << ":"
                        << ci->ProgDesc << "(" << ci->ProgIf << ")"  << eol;
                }
                else
                {
                    // Don't know...
                    out << "   NOT REGISTERED: Class: " << mainclass << ", Sub: "
                        << subclass << ", Function: " << progif << eol;

                    // At least the main/sub class and progif will be useful later on
                    ci = new PCIClassInfo();
                    ci->BaseClass = mainclass;
                    ci->BaseDesc = "Unknown";
                    ci->SubClass = subclass;
                    ci->SubDesc = "Unknown";
                    ci->ProgIf = progif;
                    ci->ProgDesc = "Unknown";
                }

                // We don't know the actual type yet...
                PCIDevice* newDevice = 0;

                /* Traverse a bridge? */

                if(mainclass == PCI_BASE_CLASS_BRIDGE && subclass == PCI_SUBCLASS_BRIDGE_PCI)
                {
                    PCIConfig::write(bus->busNum, dev, function, 27, 1, 0xFF);

                    // Read bus number
                    PCIConfig::read(bus->busNum, dev, function,
                                    PCI_CONFIG_DWORD_BASE_ADDR_2, 4, &val);

                    int primary = val & 0xFF;
                    int secondary = (val >> 8) & 0xFF;
                    int subord = (val >> 16) & 0xFF;
                    out << "   PCI-PCI Bridge: [Primary=" << primary << ", Secondary=" << secondary
                        << ", Subordinate=" << subord << "]" << eol;

                    // Update PCI object model
                    PCIBus* secondaryBus = new PCIBus(secondary);
                    bus->children.add(secondaryBus);

                    PCIBridge *bridge = new PCIBridge();
                    bridge->primary = bus;
                    bridge->secondary = secondaryBus;

                    newDevice = bridge;

                    // Continue probing the pci hierarchy
                    enumerateBus(secondaryBus);
                }
                else
                {
                    newDevice = new PCIDevice();
                }


                /* Base addresses */

                int iobaseIndex = 0;
                int membaseIndex = 0;
                for(int reg=PCI_CONFIG_DWORD_BASE_ADDR_0; reg <= PCI_CONFIG_DWORD_BASE_ADDR_5; reg++)
                {
                    // By uncommenting the following, the register is updated with info on required
                    // memory or io space length (see pp 384 of PCI System Archicture). For now, we
                    // just use the BIOS assigned values.
                    //PCIConfig::write(bus->busNum, dev, function, reg, 4, (uint32)~0);

                    // Read base address info (address bases)
                    PCIConfig::read(bus->busNum, dev, function, reg, 4, &val);

                    if(val != 0)
                    {
                        int space = val & 1; // 0=mem, 1=i/o

                        int decoder = val >> 1 & 0x03; // 00 = 32bit, 10=64-bit, 01=reserverd, 11=reserved
                        int prefetchable = (val & PCI_CONFIG_BIT_MEM_PREFETCHABLE) == PCI_CONFIG_BIT_MEM_PREFETCHABLE;
                        paddr base = val & PCI_BASEADDRESS_MASK;

                        // Query size of memory / IO required, and write back the original BIOS assigned value
                        uint32 size;
                        PCIConfig::write(bus->busNum, dev, function, reg, 4, (uint32)~0);
                        PCIConfig::read(bus->busNum, dev, function, reg, 4, &size);
                        PCIConfig::write(bus->busNum, dev, function, reg, 4, (uint32)val);

                        // Amount of mem or i/o space required by this decoder; 2^bit (this is also the alignment,
                        // per PCI spec)
                        paddr sizebase = size & PCI_BASEADDRESS_MASK;
                        int bit = Bits::firstSet((uint32)sizebase);
                        int len = (base == 0) ? 0 : Bits::valueOf(bit);

                        String spaceDesc("Memory");
                        if(space == 1)
                        {
                            spaceDesc = "I/O";

                            newDevice->setIOBaseAddress(iobaseIndex, base, len);
                            iobaseIndex++;
                        }
                        else
                        {
                            newDevice->setMemoryBaseAddress(membaseIndex, base, len);
                            membaseIndex++;
                        }

                        out <<  "   Base " << (reg-PCI_CONFIG_DWORD_BASE_ADDR_0) << ": " << H(base)
                            <<  " (Type=" << spaceDesc << ", Len=" << len <<", Prefetch=" << prefetchable
                            << ", Decoder=" << decoder << ")" << eol;
                    }
                }// base addresses

                /* PCI capabilities; see [B 1 pp 392] */

                uint32 pciStatus;
                PCIConfig::read(bus->busNum, dev, function, (PCI_CONFIG_STATUS/2), 2, &pciStatus);

                if(pciStatus & PCI_STATUS_CAPABILITY_LIST)
                {
                    // Read capabilities pointer. The positition is from byte
                    // 64 (after the 16 DWORD header) and 48 DWORDS onwards.
                    // I.e. the capabilities area is 48*4=192 bytes.
                    uint32 capPtr;
                    PCIConfig::read(bus->busNum, dev, function, PCI_CONFIG_CAPABILITIES, 1, &capPtr);

                    // Run through capabilities by following the Next Capability
                    // pointer; which is presumably 0xff or 0x00 if there is no
                    // next capability (need to check the actual spec here)
                    for(int capPos=0; capPtr != 0xff && capPtr >=64 && capPos < PCI_CAPABILITY_SIZE_DWORDS; capPos++)
                    {
                        // Mask off lower two bits per spec (should be hardwired
                        // to 0, but never know with buggy HW)
                        capPtr &= ~3;
                        out << "   Capabilities ptr: " << capPtr << ", ";

                        // Read capability ID (byte)
                        uint32 capId;
                        PCIConfig::read(bus->busNum, dev, function, capPtr, 1, &capId);
                        PCICapability* cap = null;

                        out << "ID: " << capId;
                        if(capId == 0x00)
                            out << " (Reserved)";
                        else if(capId == 0x01)
                            out << " (Power Management)";
                        else if(capId == 0x02)
                        {
                            PCICapabilityAGP* agpcap = new PCICapabilityAGP();
                            uint32 agpVer;
                            PCIConfig::read(bus->busNum, dev, function, capPtr+2, 1, &agpVer);

                            agpcap->minorRev = agpVer & 0xf;
                            agpcap->majorRev = (agpVer >> 4) & 0xf;
                            out << " (AGP " << agpcap->majorRev << "."<< agpcap->minorRev <<")";

                            cap = agpcap;
                        }
                        else if(capId == 0x04)
                            out << " (Slot ID)";
                        else if(capId == 0x05)
                            out << " (MSI)";
                        else if(capId == 0x06)
                            out << " (CompactPCI Hotswap)";

                        out << eol;

                        // Add the capability; if not a specific capability has been created, create a generic one
                        if(!cap)
                        {
                            cap = new PCICapability();
                        }

                        cap->id = capId;
                        newDevice->addCapability(cap);

                        // Next capability ptr
                        PCIConfig::read(bus->busNum, dev, function, capPtr+1, 1, &capPtr);
                    }

                }// capabilities

                // Set info and add the device to the current bus.
                newDevice->setBus(bus);

                newDevice->classInfo = ci;
                newDevice->vendorInfo = vendorInfo;
                newDevice->deviceInfo = devInfo;

                // Per-bus identification for later config access
                newDevice->deviceNum = dev;
                newDevice->functionNum = function;

                // The bus got itself a new device
                bus->devices.add(newDevice);

                // NE2000 NIC
                if(newDevice->deviceInfo->VenId == 0x10EC &&
                   newDevice->deviceInfo->DevId == 0x8029)
                {
                    newDevice->type = DEVTYPE_NIC;
                    newDevice->setIrq(line, pin);

                    NE2KDriver* nic = NE2KDriver::getInstance(newDevice);
                }
                // RTL8139D NIC
                else if(newDevice->deviceInfo->VenId == 0x10EC &&
                        newDevice->deviceInfo->DevId == 0x8139)
                {
                    newDevice->type = DEVTYPE_NIC;
                    newDevice->setIrq(line, pin);

                    RTL8139DDriver* nic = RTL8139DDriver::getInstance(newDevice);
                }
                // PCnet32 II NIC
                else if(newDevice->deviceInfo->VenId == 0x1022 &&
                        newDevice->deviceInfo->DevId == 0x2000)
                {
                    newDevice->type = DEVTYPE_NIC;
                    newDevice->setIrq(line, pin);

                    PCNet32::PCNet32Driver* nic = PCNet32::PCNet32Driver::getInstance(newDevice);
                }
                // VMWARE Display Driver (check for both PCI device id's we know about - 405 is the newer one)
                else if(newDevice->deviceInfo->VenId == 0x15AD &&
                       (newDevice->deviceInfo->DevId == 0x0405 ||
                        newDevice->deviceInfo->DevId == 0x0710))
                {
                    newDevice->type = DEVTYPE_DISPLAY;

                    Node* dev = Kernel::getNamespace()->find("/dev");
                    // TODO: there may be more than one card; give it an id
                    newDevice->setName(L"vmware");
                    dev->addChild(newDevice);
                }

                // TODO: Add device to per-class indexed lookup table so
                // drivers, etc can look it up quickly This means there should
                // be a bus->addDevice that adds it to the necessary places.
            }
        }
    }
}

/** Add a capability */
void PCIDevice::addCapability(PCICapability* cap)
{
    this->caps.add(cap);
}

/**
 * Find a capability
 *
 * @id PCI designated capability id
 * @returns A capability, or NULL if not found
 */
PCICapability* PCIDevice::findCapability(uint8 id)
{
    return 0;
}
