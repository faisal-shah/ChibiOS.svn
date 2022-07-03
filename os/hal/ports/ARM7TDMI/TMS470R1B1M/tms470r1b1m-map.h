#pragma once

/** This is the total number of interrupts on the processor (CIM) */
#define TOTAL_INT_LINES     (32)
/** This is the total number of interrupts in the system (IEM) */
#define TOTAL_SYS_INTS      (64)
/**
 * This is the value that represents interrupt values that have not been registered.
 * It is using the lowest priority.  If we ever need 32 interrupts then this will need
 * to be done a different way
 */
#define PRIORITY_NOT_ASSIGNED 	(31)

/** Structure for DMA registers */
typedef volatile struct {
    uint32_t DMACC0;
    uint32_t DMACC1;
    uint32_t DMAS;
    uint32_t DMAIO0;
    uint32_t DMAIO1;
    uint32_t DMACPS;
    uint32_t DMACPSC;
    uint32_t DMAGC;
    uint32_t DMAGD;
    uint32_t DMAAC;
    uint32_t DMACCP0;
    uint32_t DMACCP1;
    uint32_t DMACCP2;
    uint32_t DMACCP3;
} Dmaregs_TypeDef;

/** Structure for DMA control register */
typedef volatile struct {
	uint32_t DMAC;
	uint32_t DMASA;
	uint32_t DMADA;
	uint32_t DMATC;
} DmaCtrlreg_TypeDef;

/** Structure for watchdog registers */
typedef volatile struct {
    uint32_t DWCTRL;
    uint32_t DWPRLD;
    uint32_t DWKEY;
    uint32_t DWCNTR;
} Watchdog_TypeDef;

/** Structure for memory protection unit registers */
typedef volatile struct {
    uint32_t MPUAHR0;
    uint32_t MPUALR0;
    uint32_t MPUAHR1;
    uint32_t MPUALR1;
    uint32_t MPUAHR2;
    uint32_t MPUALR2;
    uint32_t MPUAHR3;
    uint32_t MPUALR3;
    uint32_t MPUCTRL;
} Mpuregs_TypeDef;

/** Structure for interrupt expansion module registers */
typedef volatile struct {
    uint32_t INTPEND0;
    uint32_t INTPEND1;
    uint32_t reserved[6];
    uint8_t INTCTRL[TOTAL_SYS_INTS];
} Iemregs_TypeDef;

/** Structure for system registers */
typedef volatile struct {
    uint32_t SMCR0;
    uint32_t SMCR1;
    uint32_t SMCR2;
    uint32_t SMCR3;
    uint32_t SMCR4;
    uint32_t SMCR5;
    uint32_t SMCR6;
    uint32_t SMCR7;
    uint32_t SMCR8;
    uint32_t SMCR9;
    uint32_t reserved;
    uint32_t WCR0;
    uint32_t PCR;
    uint32_t PLR;
    uint32_t PPROT;
} Sysregs_TypeDef;

/** Structure for memory base address registers */
typedef volatile struct {
    uint32_t MFBAHR0;
    uint32_t MFBALR0;
    uint32_t MFBAHR1;
    uint32_t MFBALR1;
    uint32_t MFBAHR2;
    uint32_t MFBALR2;
    uint32_t MFBAHR3;
    uint32_t MFBALR3;
    uint32_t MFBAHR4;
    uint32_t MFBALR4;
    uint32_t MFBAHR5;
    uint32_t MFBALR5;
    uint32_t MCBAHR0;
    uint32_t MCBALR0;
    uint32_t MCBAHR1;
    uint32_t MCBALR1;
    uint32_t MCBAHR2;
    uint32_t MCBALR2;
    uint32_t MCBAHR3;
    uint32_t MCBALR3;
    uint32_t MCBAHR4;
    uint32_t MCBALR4;
    uint32_t MCBAHR5;
    uint32_t MCBALR5;
} Memregs_TypeDef;

/** Structure for interrupt registers */
typedef volatile struct {
    uint32_t RTICNTR;
    uint32_t RTIPCTL;
    uint32_t RTICNTL;
    uint32_t WKEY;
    uint32_t RTICMP1;
    uint32_t RTICMP2;
    uint32_t RTICINT;
    uint32_t RTICNTEN;
    uint32_t IRQIVEC;
    uint32_t FIQIVEC;
    uint32_t CIMIVEC;
    uint32_t FIRQPR;
    uint32_t INTREQ;
    uint32_t REQMASK;
} Intregs_TypeDef;

/** Structure for PSA control registers */
typedef volatile struct {
	uint32_t CPU;
	uint32_t RESERVED[3];
	uint32_t ENABLE;
} Psaregs_TypeDef;

/** Structure for system control registers */
typedef volatile struct {
    uint32_t CLKCNTL;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t GLBCTRL;
    uint32_t SYSECR;
    uint32_t SYSESR;
    uint32_t ARBTESR;
    uint32_t GLBSTAT;
    uint32_t DEV;
    uint32_t reserved3;
    uint32_t SSIF;
    uint32_t SSIR;
} Syscntl_TypeDef;

/** Structure for I2C module registers */
typedef volatile struct {
	uint32_t OAR;
	uint32_t IMR;
	uint32_t SR;
	uint32_t CKL;
	uint32_t CKH;
	uint32_t CNT;
	uint32_t DRR;
	uint32_t SAR;
	uint32_t DXR;
	uint32_t MDR;
	uint32_t IVR;
	uint32_t EMR;
	uint32_t PSC;
	uint32_t DIR;
	uint32_t DOUT;
	uint32_t DIN;
	uint32_t reserved[2];
	uint32_t PFNC;
	uint32_t PDIR;
	uint32_t DINV2_X; //probably need to rename v2.x
	uint32_t DOUTV2_X; //probably need to rename v2.x
	uint32_t DSET;
	uint32_t DCLR;
	uint32_t reserved2;
	uint32_t PID1;
	uint32_t PID2;
} I2C_TypeDef;

/** Structure for SPI module registers */
typedef volatile struct {
    uint32_t CTRL1;
    uint32_t CTRL2;
    uint32_t CTRL3;
    uint32_t DAT0;
    uint32_t DAT1;
    uint32_t BUF;
    uint32_t EMU;
    uint32_t PC1;
    uint32_t PC2;
    uint32_t PC3;
    uint32_t PC4;
    uint32_t PC5;
    uint32_t PC6;
} SPI_TypeDef;

/** Structure for SCI module registers */
typedef volatile struct {
    uint32_t CCR;
    uint32_t CTL1;
    uint32_t CTL2;
    uint32_t CTL3;
    uint32_t RXST;
    uint32_t HBAUD;
    uint32_t MBAUD;
    uint32_t LBAUD;
    uint32_t RXEMU;
    uint32_t RXBUF;
    uint32_t TXBUF;
    uint32_t PC1;
    uint32_t PC2;
    uint32_t PC3;
} SCI_TypeDef;

/** Structure for MibADC ADDR register*/
typedef volatile struct {
	uint32_t ADDR;
	uint32_t ADEMDR;
} ADDR_Typedef;

/**  */
typedef volatile struct {
	ADDR_Typedef ADDR[16];
} ADCCOMP_Typedef;

/** Structure for MibADC */
typedef volatile struct{
	uint32_t reserved;
	uint32_t ADBUFE[8];
	uint32_t ADBUF1[8];
	uint32_t ADBUF2[8];
	uint32_t reserved1[4];
	uint32_t ADEMBUFE;
	uint32_t ADEMBUF1;
	uint32_t ADEMBUF2;
} ADCBUF_Typedef;

/** */
typedef union{
	ADCCOMP_Typedef COMP;
	ADCBUF_Typedef BUF;
} ADCCONFIG_Typedef;

/** Structure for MibADC module registers */
typedef volatile struct {
	uint32_t ADCR1;
	uint32_t ADCR2;
	uint32_t ADSR;
	uint32_t ADEISR;
	uint32_t ADISR1;
	uint32_t ADISR2;
	uint32_t ADCALR;
	ADCCONFIG_Typedef CONFIG;
	uint32_t ADINR;
	uint32_t ADPCR;
	uint32_t reserved2[3];
	uint32_t ADSAMPEV;
	uint32_t ADSAMP1;
	uint32_t ADSAMP2;
	uint32_t ADBCR1;
	uint32_t ADBCR2;
	uint32_t ADBCR3;
	uint32_t ADBUFST;
	int32_t ADTHREV;
	int32_t ADTHRG1;
	int32_t ADTHRG2;
	uint32_t ADEVTSRC;
} MIBADC_Typedef;

/** Structure for GIO module registers */
typedef volatile struct {
    uint32_t PWDN;
    uint32_t ENA1;
    uint32_t POL1;
    uint32_t FLG1;
    uint32_t PRY1;
    uint32_t OFFA;
    uint32_t EMUA;
    uint32_t OFFB;
    uint32_t EMUB;
    uint32_t reserved[0x28];
    uint32_t ENA2;
    uint32_t POL2;
    uint32_t FLG2;
    uint32_t PRY2;
} GIO_TypeDef;

/** Structure for GIO port registers */
typedef volatile struct {
    uint32_t DIR;
    uint32_t DIN;
    uint32_t DOUT;
    uint32_t SET;
    uint32_t CLR;
} GIOPort_TypeDef;

/** Structure for HECC registers */
typedef volatile struct {
	uint32_t CANME;
	uint32_t CANMD;
	uint32_t CANTRS;
	uint32_t CANTRR;
	uint32_t CANTA;
	uint32_t CANAA;
	uint32_t CANRMP;
	uint32_t CANRML;
	uint32_t CANRFP;
	uint32_t CANGAM;
	uint32_t CANMC;
	uint32_t CANBTC;
	uint32_t CANES;
	uint32_t CANTEC;
	uint32_t CANREC;
	uint32_t CANGIF0;
	uint32_t CANGIM;
	uint32_t CANGIF1;
	uint32_t CANMIM;
	uint32_t CANMIL;
	uint32_t CANOPC;
	uint32_t CANTIOC;
	uint32_t CANRIOC;
	uint32_t LNT;
	uint32_t TOC;
	uint32_t TOS;
	uint32_t reserved[6];
	uint32_t LAM[32];
	uint32_t MOTS[32];
	uint32_t MOTO[32];
} HECC_TypeDef;

/** Structure for HECC mailbox */
typedef volatile struct {
	uint32_t MID;
	uint32_t MCF;
	uint8_t MD[8];
} HECCBox_TypeDef;

/** Structure for HECC RAM */
typedef volatile struct {
	HECCBox_TypeDef MB[32];
} HECCRAM_TypeDef;

/** Structure for 32 bit flash registers */
typedef volatile struct {
	uint32_t REGOPT;
	uint32_t reserved;
	uint32_t BBUSY;
	uint32_t PKEY;
} Flash32_Typedef;

/** Structure for 16 bit flash registers */
typedef volatile struct {
	uint32_t BAC1;
	uint32_t BAC2;
	uint32_t BSEA;
	uint32_t BSEB;
	uint32_t BRDY;
	uint32_t reserved[3835];
	uint32_t MAC1;
	uint32_t MAC2;
	uint32_t PAGP;
	uint32_t MSTAT;
} Flash16_Typedef;

/** Structure for high end timer module registers */
typedef volatile struct {
    uint32_t GCR;
    uint32_t PFR;
    uint32_t ADDR;
    uint32_t OFF1;
    uint32_t OFF2;
    uint32_t EXC1;
    uint32_t EXC2;
    uint32_t PRY;
    uint32_t FLG;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t HRSH;
    uint32_t XOR;
    uint32_t DIR;
    uint32_t DIN;
    uint32_t DOUT;
    uint32_t DSET;
    uint32_t DCLR;
} HET_TypeDef;


/** Structure for expansion bus registers */
typedef volatile struct {
    uint32_t EBDMACR;
    uint32_t EBRWCR;
    uint32_t EBACR1;
    uint32_t EBDCR;
    uint32_t EBADCR;
    uint32_t EBACR2;
    uint32_t EBACR3;
    uint32_t EBMCR1;
} ExpansionBus_TypeDef;

/* Base addresses of system modules */
#define FLASH16_BASE	(0xFFE88000)
#define FLASH32_BASE	(0xFFE89C00)
#define MPUREGS_BASE	(0xFFFF4000)
#define MSMREGS_BASE	(0xFFFFF700)
#define DMARAM_BASE		(0xFFFFF800)
#define IEMREGS_BASE    (0xFFFFFC00)
#define SYSREGS_BASE    (0xFFFFFD00)
#define MEMREGS_BASE    (0xFFFFFE00)
#define DMAREGS_BASE	(0xFFFFFE80)
#define INTREGS_BASE    (0xFFFFFF00)
#define PSAREGS_BASE	(0xFFFFFF40)
#define	WATCHDOG_BASE	(0xFFFFFF60)
#define SYSCNTL_BASE    (0xFFFFFFD0)

/* Base addresses of peripheral modules */
#define SPI2_BASE       (0xFFF7D400)
#define I2C5_BASE		(0XFFF7D500)
#define I2C1_BASE		(0XFFF7D800)
#define I2C2_BASE		(0XFFF7D900)
#define I2C3_BASE		(0XFFF7DA00)
#define I2C4_BASE		(0XFFF7DB00)
#define HECC1RAM_BASE   (0xFFF7E400)
#define HECC2RAM_BASE   (0xFFF7E600)
#define HECC1_BASE      (0xFFF7E800)
#define HECC2_BASE      (0xFFF7EA00)
#define GIO_BASE        (0xFFF7EC00)
#define GIOA_BASE       (0xFFF7EC24)
#define GIOB_BASE       (0xFFF7EC38)
#define GIOC_BASE       (0xFFF7EC4C)
#define GIOD_BASE       (0xFFF7EC60)
#define GIOE_BASE       (0xFFF7EC74)
#define GIOF_BASE       (0xFFF7EC88)
#define GIOG_BASE       (0xFFF7EC9C)
#define GIOH_BASE       (0xFFF7ECB0)
#define	EBM_BASE		(0xFFF7ED00)
#define MIBADC_BASE		(0xFFF7F000)
#define SCI1_BASE       (0xFFF7F400)
#define SCI2_BASE       (0xFFF7F500)
#define SCI3_BASE       (0xFFF7F600)
#define SPI1_BASE       (0xFFF7F800)
#define HET_BASE        (0xFFF7FC00)
#define HETRAM_BASE	    (0x00400000)

/* Defines for system module registers */
#define FLASH32			((Flash32_Typedef *)FLASH32_BASE)
#define FLASH16			((Flash16_Typedef *)FLASH16_BASE)
#define MPUREGS         ((Mpuregs_TypeDef *)MPUREGS_BASE)
#define DMACTRL			((DmaCtrlreg_TypeDef *)DMARAM_BASE)
#define IEMREGS         ((Iemregs_TypeDef *)IEMREGS_BASE)
#define SYSREGS         ((Sysregs_TypeDef *)SYSREGS_BASE)
#define MEMREGS         ((Memregs_TypeDef *)MEMREGS_BASE)
#define DMAREGS         ((Dmaregs_TypeDef *)DMAREGS_BASE)
#define INTREGS         ((Intregs_TypeDef *)INTREGS_BASE)
#define PSAREGS			((Psaregs_TypeDef *)PSAREGS_BASE)
#define WATCHDOG        ((Watchdog_TypeDef *)WATCHDOG_BASE)
#define SYSCNTL         ((Syscntl_TypeDef *)SYSCNTL_BASE)

/* Defines for peripheral module registers */
#define HECC1           ((HECC_TypeDef *)HECC1_BASE)
#define HECC2           ((HECC_TypeDef *)HECC2_BASE)
#define HECC1RAM        ((HECCBox_TypeDef *)HECC1RAM_BASE)
#define HECC2RAM        ((HECCBox_TypeDef *)HECC2RAM_BASE)
#define GIO             ((GIO_TypeDef *)GIO_BASE)
#define GIOA            ((GIOPort_TypeDef *)GIOA_BASE)
#define GIOB            ((GIOPort_TypeDef *)GIOB_BASE)
#define GIOC            ((GIOPort_TypeDef *)GIOC_BASE)
#define GIOD            ((GIOPort_TypeDef *)GIOD_BASE)
#define GIOE            ((GIOPort_TypeDef *)GIOE_BASE)
#define GIOF            ((GIOPort_TypeDef *)GIOF_BASE)
#define GIOG            ((GIOPort_TypeDef *)GIOG_BASE)
#define GIOH            ((GIOPort_TypeDef *)GIOH_BASE)
#define MIBADC			((MIBADC_Typedef *)MIBADC_BASE)
#define SCI1            ((SCI_TypeDef *)SCI1_BASE)
#define SCI2            ((SCI_TypeDef *)SCI2_BASE)
#define SCI3            ((SCI_TypeDef *)SCI3_BASE)
#define SPI1            ((SPI_TypeDef *)SPI1_BASE)
#define SPI2            ((SPI_TypeDef *)SPI2_BASE)
#define I2C1			((I2C_TypeDef *)I2C1_BASE)
#define I2C2			((I2C_TypeDef *)I2C2_BASE)
#define I2C3			((I2C_TypeDef *)I2C3_BASE)
#define I2C4			((I2C_TypeDef *)I2C4_BASE)
#define I2C5			((I2C_TypeDef *)I2C5_BASE)
#define HET             ((HET_TypeDef *)HET_BASE)
#define HETRAM			((HetCtrl_TypeDef *)HETRAM_BASE)
#define EBM				((ExpansionBus_TypeDef *)EBM_BASE)
