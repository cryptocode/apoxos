/**
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * ATA definitions
 *
 * The "T13/1532D Volume 1 Revision 4b" specification document is the primary reference.
 *
 * \defgroup ATA
 * @{
 */

#ifndef _APOX_ATA_H_
#define _APOX_ATA_H_

#include <kernel/libc/std.h>


/* Timeouts */

/** Default timeout is 500 ms; but us unit */
#define ATA_TIMEOUT_US_DEFAULT          1000*500

#define ATA_TIMEOUT_250_MS               250
#define ATA_TIMEOUT_500_MS               500
#define ATA_TIMEOUT_1000_MS              1000

/** Data request timeout (device should be ready to xfer data within this limit) */
#define ATA_TIMEOUT_DRQ_MS              500

/** Wait up to 15 sec for irq's to happend, by default */
#define ATA_TIMEOUT_IRQ_MS              15000

/** Timeout for IDENTIFY */
#define ATA_TIMEOUT_ATA_IDENTIFY_MS     10000

/** The CD/DVD door may not be closed; hence the long timeout */
#define ATA_TIMEOUT_ATAPI_IDENTIFY_MS   40000



/* Commands */

/** Execute internal diagnostics and place signature in command block */
#define ATA_CMD_EXEC_DEVICE_DIAGNOSTIC  0x90

/** Identifies drives */
#define ATA_CMD_IDENTIFY_DEVICE         0xEC

/** Eject the media */
#define ATA_CMD_MEDIA_EJECT             0xED

/** Packet version of the identify command */
#define ATA_CMD_PIDENTIFY_DEVICE        0xA1

/** Get status */
#define ATA_CMD_STATUS                  0x07

/** Reset packet device */
#define ATA_CMD_DEVICE_RESET            0x08

/** Read 1-256 sectors, CHS, LBA28 */
#define ATA_CMD_READ                    0x20

/** Read 1-65K sectors, LBA48 */
#define ATA_CMD_READ_EXT                0x24

/** Read 1-256 sectors, multiple per interrupt, CHS?, LBA28 */
#define ATA_CMD_READ_MULTIPLE           0xC4

/** Read 1-65K sectors, multiple per interrupt, LBA48 */
#define ATA_CMD_READ_MULTIPLE_EXT       0x29

/** Write 1-256 sectors, CHS, LBA28 */
#define ATA_CMD_WRITE                   0x30

/** Write 1-65K sectors, LBA48 */
#define ATA_CMD_WRITE_EXT       0x34

/** Write 1-256 sectors, multiple per interrupt, CHS?, LBA28 */
#define ATA_CMD_WRITE_MULTIPLE          0xC5

/** Write 1-65K sectors, multiple per interrupt, LBA48 */
#define ATA_CMD_WRITE_MULTIPLE_EXT      0x39

/* Status bits */

/** Error flag */
#define ATA_STATUS_ERR              0x01
#define ATA_STATUS_INDEX            0x02
#define ATA_STATUS_CORRECTED        0x04
/** Data request - the drive is ready to xfer to/from the data port*/
#define ATA_STATUS_DRQ              0x08
#define ATA_STATUS_DSC              0x10
/** Device fault */
#define ATA_STATUS_DF               0x20
/** Drive ready to accept commands */
#define ATA_STATUS_DRDY             0x40
/** Busy flag -> do not touch command registers */
#define ATA_STATUS_BSY              0x80


/*
 * Command block registers. Some registers means different things in different
 * contexts; hence the various alias definitions.
 */

#define ATA_REG_DATA                0x0000
#define ATA_REG_ERROR               0x0001
#define ATA_REG_FEATURE             0x0001
#define ATA_REG_SECTORCOUNT         0x0002
#define ATA_REG_LBA_LOW             0x0003
#define ATA_REG_LBA_MID             0x0004
#define ATA_REG_LBA_HIGH            0x0005
/** Select device (and heads when CHS addressing) */
#define ATA_REG_DEVSEL              0x0006
/** Command registers (same as status register) */
#define ATA_REG_COMMAND             0x0007
/** Status register */
#define ATA_REG_STATUS              0x0007

/** Same as the status register, but doesn't touch flags; yields 0x3F6/0x3F7 when added to controller base */
#define ATA_REG_ALT_STATUS          0x0206

//?
//#define ATA_REG_RELEASED            0x0008

/*
 * Base command block registers (in legacy I/O space; PCI mode may override this)
 */

#define ATA_IOBASE_CONTROLLER_1     0x01F0
#define ATA_IOBASE_CONTROLLER_2     0x0170

// Yields 0x03F6 and 0x0376 when added to the controller's IO base.
// This is the Alternate Status on reads and Device Control on writes (aka DOR - Digital Otput Register)
#define ATA_IOBASE_CONTROL_REG      0x0206

/* Reset device by writing this to DOR */
#define ATA_CONTROL_RESET           0x04

/* Enable/disable bit when writing to DOR */
#define ATA_CONTROL_IRQ_BLOCK       0x02

#endif // _APOX_ATA_H_

/**@}*/
