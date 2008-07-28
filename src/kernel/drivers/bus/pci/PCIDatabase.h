/**
 * Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 *
 * \defgroup Drivers
 * @{
 */

#ifndef _APOX_PCI_DATABASE_H_
#define _APOX_PCI_DATABASE_H_

#include <kernel/libc/std.h>
#include <kernel/drivers/bus/pci/PCITables.h>

/**
 * PCI database access. The database is actually structs of vendor/device info
 * derived from pcidatabase.com.
 *
 * \note Singleton
 */
class PCIDatabase
{
    public:

        static PCIVendorInfo* getVendorById(uint16 vendorId);
        static PCIDeviceInfo* getDeviceById(uint16 vendorId, uint16 devId);
        static PCIClassInfo* getClassInfo(uint8 classCode, uint8 subclassCode, uint8 progIF);
        static PCIClassInfo* getClassInfo(uint8 classCode, uint8 subclassCode);
        static PCIClassInfo* getClassInfo(uint32 classCode);
};

#endif // _APOX_PCI_DATABASE_H_

/**@}*/
