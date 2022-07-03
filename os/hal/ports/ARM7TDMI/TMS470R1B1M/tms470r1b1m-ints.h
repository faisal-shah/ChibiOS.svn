/**
 * @file    tms470r1b1m-ints.h
 * @brief   Defines for the TMS470R1B1M interrupts
 */
#pragma once

/* These are the IEM interrupt locations (default CIM locations) */ 
#define IEMNUM_SPI1         (0)
#define IEMNUM_COMP2        (1)
#define IEMNUM_COMP1        (2)
#define IEMNUM_TAP          (3)
#define IEMNUM_SPI2         (4)
#define IEMNUM_GIOA         (5)
#define IEMNUM_RESERVED1	(6)
#define IEMNUM_HET1         (7)
#define IEMNUM_I2C1         (8)
#define IEMNUM_SCI1ERR      (9)     //These two are intentionally the same
#define IEMNUM_SCI2ERR      (9)
#define IEMNUM_SCI1RX       (10)
#define IEMNUM_RESERVED2	(11)
#define IEMNUM_I2C2         (12)
#define IEMNUM_HECC1A       (13)
#define IEMNUM_SCCA         (14)
#define IEMNUM_RESERVED3	(15)
#define IEMNUM_MIBADCEVENT  (16)
#define IEMNUM_SCI2RX       (17)
#define IEMNUM_DMA0         (18)
#define IEMNUM_I2C3         (19)
#define IEMNUM_SCI1TX       (20)
#define IEMNUM_SWI          (21)
#define IEMNUM_RESERVED4	(22)
#define IEMNUM_HET2         (23)
#define IEMNUM_HECC1B       (24)
#define IEMNUM_SCCB         (25)
#define IEMNUM_SCI2TX       (26)
#define IEMNUM_MIBADCGRP1   (27)
#define IEMNUM_DMA1         (28)
#define IEMNUM_GIOB         (29)
#define IEMNUM_MIBADCGRP2   (30)
#define IEMNUM_SCI3ERR      (31)
//IEMs 32-37 are reserved
#define IEMNUM_HECC2A       (38)
#define IEMNUM_HECC2B       (39)
#define IEMNUM_SCI3RX       (40)
#define IEMNUM_SCI3TX       (41)
#define IEMNUM_I2C4         (42)
#define IEMNUM_I2C5         (43)

/**
 * @brief Low priority external interrupt handler
 * @return None
 */
void GIOB_IRQHandler(void);
