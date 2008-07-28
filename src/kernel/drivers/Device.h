/*
 * Apox Operating System
 * Copyright (c) 2005, 2006, 2007 cryptocode
 */

#ifndef _APOX_DEVICE_H_
#define _APOX_DEVICE_H_

#include <kernel/init/context.h>
#include <kernel/Component.h>
#include <kernel/unicode/Unistring.h>
#include <kernel/io/Channel.h>
#include <kernel/adt/List.h>
#include <kernel/adt/PropertyBag.h>
#include <kernel/Node.h>

/** Device types */
enum DeviceType
{
    DEVTYPE_UNKNOWN=0,
    DEVTYPE_SYSTEM,
    DEVTYPE_VFS,
    DEVTYPE_NIC,
    DEVTYPE_DISPLAY,
};

/** Device subtypes */
enum DeviceSubType
{
    DEVSUBTYPE_UKNOWN,
    DEVSUBTYPE_SERIAL,
    DEVSUBTYPE_PARALLELL,
    DEVSUBTYPE_ATAPI,
    DEVSUBTYPE_PCI_DEVICE,
    DEVSUBTYPE_PCI_BRIDGE,
    DEVSUBTYPE_USB,
    DEVSUBTYPE_BLUETOOTH,
};

/** Device status */
enum DeviceStatus
{
    DEVSTATUS_UNKNOWN,
    DEVSTATUS_STOPPED,
    DEVSTATUS_SUSPENDED,
    DEVSTATUS_RUNNING
};

/** Device resource type */
enum DeviceResourceType
{
    DEVRES_IO,
    DEVRES_IRQ,
    DEVRES_MEM,
    DEVRES_DMAMEM
};

/**
 * A device resource is a range of I/O ports, memory or IRQ lines owned by a device.
 */
class DeviceResource
{
    public:

        DeviceResource()
        {
        }

        uint64_t from;
        uint64_t to;
        enum DeviceResourceType type;
};

/**
 * This is the base class of all bus drivers, each of which are responsible for probing
 * devices on its bus, handle new device attachment and detachments, enumerating devices for
 * shutdown and so on.
 */
class BusDriver : public Component
{
    public:

        /** C'tor */
        BusDriver()
        {}

        /** The bus driver should shutdown all devices on the bus / bus hierarchy */
        virtual void onShutdown()
        {}
};

class DeviceDriver;

/**
 * There is a device instance for each reckognized device. Note that if there
 * are several devices of the same type (such as multiple NIC's), then there
 * will be one instance for each device. Busses are considered devices.
 *
 * In general, for each device class, a subclass of Device is made, such as
 * PCIDevice and ATADevice. These contains information specific to the respective
 * type of device.
 *
 * The actual driver inherits from DeviceDriver. Note that there may be devices
 * that does not have drivers yet, and that drivers can be replaced.
 */
class Device : public Node
{
    public:

        /** C'tor */
        Device()
        {
            driver = null;
            this->irqLine = -1;
            this->irqPin = -1;
        }

        /** C'tor */
        Device(DeviceDriver* driver, const String& name, Device* parent = null)
            : Node(parent)
        {
            this->driver = driver;
            this->name = name;

            this->irqLine = -1;
            this->irqPin = -1;
        }

        /** Set IRQ */
        void setIrq(int line, int pin)
        {
            irqLine = line;
            irqPin = pin;
        }

        int getIrqLine()
        {
            return irqLine;
        }

        int getIrqPin()
        {
            return irqPin;
        }

        void setDriver(DeviceDriver* driver)
        {
            this->driver = driver;
        }

        DeviceDriver* getDriver()
        {
            return driver;
        }

/* FIXME: make protected */

        /** Who drives this device */
        DeviceDriver* driver;

        /** List of DeviceResource instances */
        List<DeviceResource*> resources;

        /** Type of device */
        enum DeviceType type;

        /** Device subtype */
        enum DeviceSubType subType;

        /** A descriptive comment */
        char* comment;

        /** Device specific data */
        void* deviceData;

    private:

        int irqLine;
        int irqPin;
};

/**
 * Base type of all device drivers. There is exactly one instance of a DeviceDriver
 * derivative per device.
 */
class DeviceDriver : public Component
{
public:

    /** C'tor */
    DeviceDriver(Device* dev = null)
    {
        this->dev = dev;
        status = DEVSTATUS_UNKNOWN;
    }

    void setDevice(Device* dev)
    {
        this->dev = dev;
    }

    Device* getDevice()
    {
        return dev;
    }

    /**
     * Called when an interrupt registered by this driver occurs. If the interrupt may
     * be shared, the driver must check for a irq pending flags on the device or equivalent.
     *
     * The default implementation bugs out, since it means the driver has registered
     * for an interrupt it doesn't handle. This is better than a pure virtual, since many
     * drivers doesn't use interrupts at all.
     *
     * @returns IRQRES_NORMAL or IRQRES_RESCHEDULE. In the latter case, a rechedule is issued ASAP.
     */
    virtual int onInterrupt(ContextState* ctx)
    {
        kassert(FALSE);
        return 0;
    }

    /** Change device status; Overriders MUST call the default method when finished to
     * update the internal status indicator. */
    virtual error_t setStatus(DeviceStatus status)
    {
        this->status = status;
        return E_OK;
    }

    DeviceStatus getStatus()
    {
        return status;
    }

    /** The device we're driving */
    Device* dev;

protected:

    /** Current status */
    DeviceStatus status;
};

#endif // _APOX_DEVICE_H_
