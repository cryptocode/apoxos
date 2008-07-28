/**
 * Apox Operating System
 * Copyright 2006, cryptocode
 *
 * \defgroup PCI
 * @{
 */

#ifndef _APOX_PCI_H_
#define _APOX_PCI_H_

#include <kernel/libc/std.h>
#include <kernel/drivers/Device.h>
#include <kernel/adt/Queue.h>
#include <kernel/adt/List.h>

/** Sentinel for unknown subordinate numbers */
#define PCI_SUBORDINARY_UNDETERMINED        0xFF

/** Primary bus number */
#define PCI_PRIMARY_BUS                     0x18

/** Secondary bus number */
#define PCI_SECONDARY_BUS                   0x19

/** Highest bus number behind the bridge */
#define PCI_SUBORDINATE_BUS                 0x1A

#define PCI_CLASS_DEVICE                    0x0A
#define PCI_CLASS_BASE                      0x0B
#define PCI_BASE_CLASS_BRIDGE               0x06

// Mainclasses of interest
#define PCI_CLASS_UKNOWN                    0x00
#define PCI_CLASS_MASS_STORAGE_CONTROLLER   0x01
#define PCI_CLASS_NETWORK_CONTROLLER        0x02
#define PCI_CLASS_DISPLAY_CONTROLLER        0x03
#define PCI_CLASS_MULTIMEDIA_CONTROLLER     0x04
#define PCI_CLASS_MEM_CONTROLLER            0x05
#define PCI_CLASS_BRIDGE_DEVICE             0x06
#define PCI_CLASS_SIMPLE_COM                0x07
#define PCI_CLASS_BASESYSTEM_PERIPHERAL     0x08
#define PCI_CLASS_INPUT_DEVICE              0x09
#define PCI_CLASS_DOCKING_STATION           0x0A
#define PCI_CLASS_CPU                       0x0B
#define PCI_CLASS_SERIALBUS_CONTROLLER      0x0C
#define PCI_CLASS_WIRELESS_CONTROLLER       0x0D
#define PCI_CLASS_INTELLIGENT_IO_CONTROLLER 0x0E
#define PCI_CLASS_SATELLITE_IO_CONTROLLER   0x0F
#define PCI_CLASS_ENCRYPTION_CONTROLLER     0x10
#define PCI_CLASS_SIGNALLING_CONTROLLER     0x11
#define PCI_CLASS_MISC                      0xFF

// Subclasses of interest
#define PCI_SUBCLASS_IDE                    0x01
#define PCI_SUBCLASS_BRIDGE_PCI             0x04

// Misc
#define PCI_CLASS_BRIDGE_HOST               0x0600
#define PCI_VENDOR_ID                       0x00

/** Host <-> PCI bridge */
#define PCI_HOST_BRIDGE                     0x060000
#define PCI_ISA_BRIDGE                      0x060100
#define PCI_EISA_BRIDGE                     0x060200

/** Class=0x06, Subclass=0x04 => PCI<->PCI bridge */
#define PCI_PCI_BRIDGE                      0x060400

#define PCI_VENDOR_ID_INTEL                 0x8086
#define PCI_VENDOR_ID_COMPAQ                0x0E11

// Mainclass + subclass shortcuts...
#define PCI_CLASS_DISPLAY_VGA               0x0300

class PCIBus;
class PCIDevice;
struct PCIClassInfo;
struct PCIDeviceInfo;
struct PCIVendorInfo;

/**
 * A device has zero or more "new" capabilities, such as AGP, power mgmt and MSI. Using the
 * id field, instances can be safely downcasted to AGPCapability, etc.
 */
class PCICapability
{
    public:

        uint8 id;
};

/** AGP capability */
class PCICapabilityAGP : public PCICapability
{
    public:

        uint8 minorRev;
        uint8 majorRev;
};

/**
 * PCI Driver
 */
class DriverPCI : public DeviceDriver
{
    public:

        /** Initialize driver */
        static DriverPCI* initialize();

        /** Create singleton and initialize the PCI subsystem*/
        static DriverPCI* getInstance();

        /** PCI IRQ routing table */
        struct PCIPirTable* pir;

        /** Root of PCI bus tree */
        PCIBus* rootBus;

    private:

        DriverPCI();

        /** Probe for PCI buses and devices  */
        void probe();

        /** Enumerate the PCI bus hierachy. Calls itself recursively for child buses.  */
        void enumerateBus(PCIBus* bus);

        /** Singleton */
        static DriverPCI* instance;
};

/** PCI Base Address (memory and IO) */
class PCIBaseAddress
{
    public:

        PCIBaseAddress()
        {
            address = 0;
            size = 0;
        }

        //int type;
        uint32 address;
        uint32 size;
};

/**
 * Represents a PCI device, including bridges.
 */
class PCIDevice : public Device
{
    public:

        /** C'tor */
        PCIDevice()
        {
            classInfo = null;
            deviceInfo = null;
            vendorInfo = null;
            bus = null;

            memset(&membar,0,sizeof(membar));
            memset(&iobar,0,sizeof(iobar));
        }

        /** Change power state of the device */
        void setPowerState(int state);

        /** Switch busmaster bit in the cmd config register */
        void setBusMaster(bool isMaster);

        /** Add a capability */
        void addCapability(PCICapability* cap);

        /** Set which PCI bus this device is attached to */
        void setBus(PCIBus* bus)
        {
            this->bus = bus;
        }

        /** Returns the PCI device this device is attached to */
        PCIBus* getBus()
        {
            return bus;
        }

        /**
         * Find a capability
         *
         * @id PCI designated capability id
         * @returns A capability, or NULL if not found
         */
        PCICapability* findCapability(uint8 id);

        PCIClassInfo* classInfo;
        PCIDeviceInfo* deviceInfo;
        PCIVendorInfo* vendorInfo;

        uint16 deviceNum;
        uint16 functionNum;

        // TODO: add flag indicating whether or not to update pci config space

        void setMemoryBaseAddress(int index, uint32 address, uint32 size)
        {
            membar[index].address = address;
            membar[index].size = size;
        }

        PCIBaseAddress& getMemoryBaseAddress(int index)
        {
            return membar[index];
        }

        void setIOBaseAddress(int index, uint32 address, uint32 size)
        {
            iobar[index].address = address;
            iobar[index].size = size;
        }

        PCIBaseAddress& getIOBaseAddress(int index)
        {
            return iobar[index];
        }

    private:

        // Up to 8 memory BAR entries
        PCIBaseAddress membar[8];

        // Up to 8 IO BAR entries
        PCIBaseAddress iobar[8];

        /** The bus we're attached to */
        PCIBus* bus;

        /** Capability list */
        List<PCICapability*> caps;
};

/**
 * A PCI bridge (to pci, isa, etc)
 */
class PCIBridge : public PCIDevice
{
    public:

        PCIBus* primary;
        PCIBus* secondary;
        PCIBus* subordinate;
};

/**
 * A PCI bus
 *
 * TODO: this is both a PCIDevice and a bus
 */
class PCIBus : public PCIDevice
{
    public:

        /** C'tor */
        PCIBus(uint8 bus)
        {
            busNum = bus;
            parent = null;
        }

        uint8 getBusNumber()
        {
            return busNum;
        }

        uint8 busNum;

        /** Parent PCI bus, if any */
        PCIBus* parent;

        /** Child PCI buses */
        List<PCIBus*> children;

        /** Devices on this particular PCI bus, including bridges */
        List<PCIDevice*> devices;
};

#endif // _APOX_PCI_H_

/**@}*/
