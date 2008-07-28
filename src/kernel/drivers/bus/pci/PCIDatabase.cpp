/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * PCI database access implementation. The "database" is actually a periodically updated
 * header files, generated on the basis of the real database at http://pcidatabase.com/.
 */

#include <kernel/drivers/bus/pci/PCIDatabase.h>

/**
 * Get vendor info
 */
PCIVendorInfo* PCIDatabase::getVendorById(uint16 vendorId)
{
    for(int i=0; i < (int) PCI_VENTABLE_LEN; i++)
    {
        if(vendorId == PciVenTable[i].VenId)
        {
            return &PciVenTable[i];
        }
    }

    return 0;
}

/**
 * Get device info
 */
PCIDeviceInfo* PCIDatabase::getDeviceById(uint16 vendorId, uint16 devId)
{
    for(int i=0; i < (int) PCI_DEVTABLE_LEN; i++)
    {
        if(vendorId == PciDevTable[i].VenId &&
           devId == PciDevTable[i].DevId)
        {
            return &PciDevTable[i];
        }
    }

    return 0;
}

/** Get class info - don't care about function */
PCIClassInfo* PCIDatabase::getClassInfo(uint8 classCode, uint8 subclassCode)
{
    for(int i=0; i < (int) PCI_CLASSCODETABLE_LEN; i++)
    {
        if(classCode == PciClassCodeTable[i].BaseClass &&
           subclassCode == PciClassCodeTable[i].SubClass)
        {
            return &PciClassCodeTable[i];
        }
    }

    return 0;

}

/**
 * Get class info
 */
PCIClassInfo* PCIDatabase::getClassInfo(uint8 classCode, uint8 subclassCode, uint8 progIF)
{
    for(int i=0; i < (int) PCI_CLASSCODETABLE_LEN; i++)
    {
        if(classCode == PciClassCodeTable[i].BaseClass &&
           subclassCode == PciClassCodeTable[i].SubClass &&
           progIF == PciClassCodeTable[i].ProgIf)
        {
            return &PciClassCodeTable[i];
        }
    }

    return 0;
}

/**
 * Get class code, using full class code and revision dword as obtained from the
 * PCI configuration header.
 *
 * \note No revision info is currently returned
 */
PCIClassInfo* PCIDatabase::getClassInfo(uint32 classCode)
{
    return getClassInfo(classCode >> 24, classCode >> 16 & 0xFF, classCode >> 8 & 0xFF);
}
