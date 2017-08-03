/**
 * Apox Operating System
 * Copyright 2006, cryptocode
 */

/*
 * PCnet register definitions. The register values are written to RAP before
 * reading or writing to the CSR or BCR data ports.
 *
 * See the data sheet from pp. 108 for details.
 */

#ifndef _APOX_DRIVERS_NIC_PCNET32REGS_H_
#define _APOX_DRIVERS_NIC_PCNET32REGS_H_


/* I/O register space offsets; assuming 32-bit wide i/o access */


/** Address PROM. First 6 bytes contains the MAC address */
#define REG_ADDRESS_PROM                0x00

/** Register Data Port */
#define REG_RDP                         0x10

/** Register Address Port; latch CSR or BCR here before accessing RDP*/
#define REG_RAP                         0x14

/** Bus configuration Data Port */
#define REG_BDP                         0x1C

/** Reading causes a chip reset */
#define REG_RESET                       0x18


/* CONTROL AND STATUS REGISTERS */


/** Controller status register. To clear an IRQ condition, read and write back CSR0. */
#define REG_CSR_STATUS                  0

/** Initialization block address, low 16 bits; only bits 15..0 are used; 31..16 are reserved. */
#define REG_CSR_IADR_LOW                1

/** Initialization block address, high 16 bits (maps to location 16) */
#define REG_CSR_IADR_HI                 2

/** Interrupt Masks and Deferral Control (maps to location 17) */
#define REG_CSR_IRQ_MASK                3

/** Test and Features Control */
#define REG_CSR_TEST_FEATURES           4

/** Extended Control and Interrupt */
#define REG_CSR_EXT_CTRL                5

/** RX/TX Encoded Ring Lengths */
#define REG_CSR_RXTX_RING_LEN           6

/** Extended Control and Interrupt 2 (this came in Am79C973/75 and is thus not supported
 *  in VMWare) */
#define REG_CSR6_EXT_CTRL2              7

/** LADR0: Logical Address Filter –- LADRF[15:0] */
#define REG_CSR_LADR0                   8

/** LADR1: Logical Address Filter –- LADRF[31:16] */
#define REG_CSR_LADR1                   9

/** LADR2: Logical Address Filter –- LADRF[47:32] */
#define REG_CSR_LADR2                   10

/** LADR3: Logical Address Filter –- LADRF[63:48] */
#define REG_CSR_LADR3                   11

/** PADR0: Physical Address Register –- PADR[15:0] */
#define REG_CSR_PADR0                   12

/** PADR1: Physical Address Register –- PADR[31:16] */
#define REG_CSR_PADR1                   13

/** PADR2: Physical Address Register –- PADR[47:32] */
#define REG_CSR_PADR2                   14

/** Mode register */
#define REG_CSR_MODE                    15

/** IADR[15:0]: Base Address of INIT Block Lower (Copy) */
#define REG_CSR_IADR_LOW_COPY           16

/** IADR[31:16]: Base Address of INIT Block Upper (Copy) */
#define REG_CSR_IADR_LOW_COPY           17

/** CRBAL: Current Receive Buffer Address Lower */
#define REG_CSR_CUR_RXBUF_LOW           18

/** CRBAU: Current Receive Buffer Address Upper */
#define REG_CSR_CUR_RXBUF_HI            19


/* STATUS REGISTER BITS */


/** Set whenever BABL, CERR, MISS or MERR is set */
#define CSR_STATUS_ERR                  15

/** Transmitter timeout error */
#define CSR_STATUS_BABL                 14

/** Collision error; just a test feature */
#define CSR_STATUS_CERR                 13

/** Set upon missed frame due to lack of rx description availability */
#define CSR_STATUS_MISS                 12

/** MERR: Memory error */
#define CSR_STATUS_MERR                 11

/** RINT: Receive interrupt */
#define CSR_STATUS_RINT                 10

/** TINT: Transmission interrupt */
#define CSR_STATUS_TINT                 9

/**
 * IDON: Initialization done (set when INIT sequence is completed).
 * When set, IDON IRQ is generated (unless masked)
 */
#define CSR_STATUS_IDON                 8

/**
 * INTR: If set, an unmasked IRQ condition has occured:
 *       BABL, EXDINT, IDON, JAB, MERR, MISS, MFCO, MPINT,
 *       RVCC, RINT, SINT, SLPINT, TINT, TXSTRT, UINT)
 */
#define CSR_STATUS_INTR                 7

/** IENA: Interrupt enable. Toggles chip interrupts on and off */
#define CSR_STATUS_IENA                 6

/** RXON: Read only.  Indicates that RX is enabled */
#define CSR_STATUS_RX_ON                5

/** TXON: Read only. Indicates that TX is enabled */
#define CSR_STATUS_TX_ON                4

/** TDMD: Transmit demand */
#define CSR_STATUS_TX_DEMAND            3

/** STOP: Set by reset or manually. Chip remains inactive until start or init bits are set. */
#define CSR_STATUS_STOP                 2

/** STRT: Start bit. Enables controller to send and receive. Writing a 1 clears the stop bit. */
#define CSR_STATUS_START                1

/** INIT: Setting init begins the initialization process,
 * including reading the initialization block. Clears the stop bit. */
#define CSR_STATUS_INIT                 0

/* BUS CONTROL REGISTERS */

/** BCR9: Full duplex control */
#define REG_BCR_FD                      9

    /** Full duplex mode bit */
    #define BCR_FD_FDEN                 0

    /** AUI port full duplex bit */
    #define BCR_FD_AUIFD                1

/** BCR18: Burst and Bus Control Register */
#define REG_BCR_BURST_AND_CONTROL       18

/** BCR20: Software style; default is 0, meaning 16-bit mode. */
#define REG_BCR_SWSTYLE                 20

/* CSR4 BITS */
#define CSR4_DMAPLUS    14
#define CSR4_APAD_XMT   11

/* CSR5 BITS */
#define CSR5_INTE_LAST_TX           14
#define CSR5_INTE_SYS               10
#define CSR5_INTE_SLEEP_DONE        9
#define CSR5_INTE_EXTD_INT          7
#define CSR5_INTE_MAGIC_PACKET      3

#endif // _APOX_DRIVERS_NIC_PCNET32REGS_H_
