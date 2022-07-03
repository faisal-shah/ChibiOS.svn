/**
 * @file    tms470r1b1m-bitdefs.h
 * @brief   Bit definitions for tms470r1b1m registers
 */
#pragma once

/*********************************** ADC *********************************/
/* ADCR1 11.1 */
#define ADC_COS				(0x8000)
#define ADC_CAL_EN			(0x2000)
#define ADC_CAL_ST			(0x1000)
#define ADC_BRIDGE_EN		(0x0800)
#define ADC_HILO			(0x0400)
#define ADC_SELF_TST		(0x0200)
#define ADC_PWR_DN			(0x0100)
#define ADC_ADC_ENA			(0x0020)
#define ADC_ACQ(x)			((0x0018) & (x < 3))
#define ADC_ACQ_2			(0)
#define ADC_ACQ_8			(1)
#define ADC_ACQ_32			(2)
#define ADC_ACQ_128			(3)
#define ADC_PS(x)			((0x0007) & x)
#define ADC_PS_1			(0)
#define ADC_PS_2			(1)
#define ADC_PS_3			(2)
#define ADC_PS_4			(3)
#define ADC_PS_5			(4)
#define ADC_PS_6			(5)
#define ADC_PS_7			(6)
#define ADC_PS_8			(7)

/* ADCR2 11.2 */
#define ADC_EV_MODE			(0x0800)
#define ADC_FRZ_EV			(0x0400)
#define ADC_ENA_EV_INT		(0x0200)
#define ADC_EV_EDG_SEL		(0x0100)
#define ADC_G1_MODE			(0x0020)
#define ADC_FRZ_G1			(0x0010)
#define ADC_ENA_G1_INT		(0x0008)
#define ADC_G2_MODE			(0x0004)
#define ADC_FRZ_G2			(0x0002)
#define ADC_ENA_G2_INT		(0x0001)


/* ADSR 11.3 */
#define ADC_G1_BUSY			(0x2000)
#define ADC_G2_BUSY			(0x1000)
#define ADC_EV_BUSY			(0x0800)
#define ADC_G1_STOP			(0x0400)
#define ADC_G2_STOP			(0x0200)
#define ADC_EV_STOP			(0x0100)
#define ADC_G1_END			(0x0004)
#define ADC_G2_END			(0x0002)
#define ADC_EV_END			(0x0001)

/* ADEISR 11.4 */
#define ADC_EV(x)			(0xFFFF & x)

/* ADISR1 11.5 */
#define ADC_G1(x)			(0xFFFF & x)

/* ADISR2 11.6 */
#define ADC_G2(x)			(0xFFFF & x)

/* ADCALR 11.7 */

/* ADDR 11.8 */
#define ADC_DTXST			(0x8000)

/* ADEMDR 11.9 */
#define ADC_EDTXST			(0x8000)

/* ADINR 11.10 */

/* ADPCR 11.11 */
#define ADC_EVT_IN			(0x0008)
#define ADC_EVT_OUT			(0x0004)
#define ADC_EVT_DIR			(0x0001)

/* ADBUFE 11.12 */
#define ADC_EV_BUF_EMPTY	(0x8000)

/* ADEMBUFE 11.13 */
//#define ADC_EV_BUF_EMPTY	(0x8000)

/* ADBUF1 11.14 */
#define ADC_G1_BUF_EMPTY	(0x8000)

/* ADEMBUF1 11.15 */
//#define ADC_G1_BUF_EMPTY	(0x8000)

/* ADBUF2 11.16 */
#define ADC_G2_BUF_EMPTY	(0x8000)

/* ADEMBUF2 11.17 */
//#define ADC_G2_BUF_EMPTY	(0x8000)

/* ADBCR1 11.18 */
#define ADC_BUFEN			(0x8000)
#define ADC_BNDA(x)			((0x7F & x) << 8)
#define ADC_BNDB(x)			(0x00FF & x)

/* ADBCR2 11.19 */
#define ADC_BNDEND			(0x0002) //TMS470R1B1x 64 word FIFO

/* ADBCR3 11.20 */
#define ADC_EV_DMAEN		(0x8000)
#define ADC_G1_DMAEN		(0x4000)
#define ADC_G2_DMAEN		(0x2000)
#define ADC_EV_OVR_INTEN 	(0x1000)
#define ADC_G1_OVR_INTEN 	(0x0800)
#define ADC_G2_OVR_INTEN 	(0x0400)
#define ADC_EV_BUF_INTEN 	(0x0200)
#define ADC_G1_BUF_INTEN 	(0x0100)
#define ADC_G2_BUF_INTEN 	(0x0080)
#define ADC_EV_8_BIT		(0x0008)
#define ADC_G1_8_BIT		(0x0004)
#define ADC_G2_8_BIT		(0x0002)
#define ADC_CHID			(0x0001)

/* ADBUFST 11.21 */
#define ADC_EV_INT_FLAG		(0x0400)
#define ADC_G1_INT_FLAG		(0x0200)
#define ADC_G2_INT_FLAG		(0x0100)
#define ADC_EV_OVR			(0x0040)
#define ADC_G1_OVR			(0x0020)
#define ADC_G2_OVR			(0x0010)
#define ADC_EV_EMPTY		(0x0004)
#define ADC_G1_EMPTY		(0x0002)
#define ADC_G2_EMPTY		(0x0001)

/* ADTHREV 11.22 */
#define ADC_EVTHR(x)		(0x03FF & x)

/* ADTHRG1 11.23 */
#define ADC_G1THR(x)		(0x03FF & x)

/* ADTHRG2 11.24 */
#define ADC_G2THR(x)		(0x03FF & x)

/* ADSAMPEV 11.25 */
#define ADC_SEN				(0x8000)
#define ADC_EVACQ(x)		(0x00FF & x)

/* ADSAMP1 11.26 */
#define ADC_G1ACQ(x)		(0x00FF & x)

/* ADSAMP2 11.27 */
#define ADC_G2ACQ(x)		(0x00FF & x)

/* ADEVTSRC 11.28 */
#define ADC_G1_ENA			(0x0100)
#define ADC_G1_EDG_SEL		(0x0080)
#define ADC_G1SRC(x)		((0x0030) & (x << 4))
#define ADC_EVSRC(x)		(0x0003 & x)
#define ADC_EVSRC_ADEVT		(0x00)

/*********************************** HET *********************************/
/* HETGCR 6.2 */
#define HET_MASTER			(0x00010000)
#define HET_64BIT_ENA		(0x00000100)
#define HET_IGNR_SBREAK		(0x00000002)
#define HET_START			(0x00000001)

/* HETEXC1 6.7 */
#define HET_OVRFL_ENA		(0x00000100)
#define HET_OVRFL_HIPRI 	(0x00000001)

/* HETEXC2 6.8 */
#define HET_OVRFL_FLG		(0x00000001)

/*********************************** I2C *********************************/
/* I2CIMR 7.2 */
#define I2C_AASEN			(0x40)
#define I2C_SCDEN			(0x20)
#define I2C_TXRDYEN			(0x10)
#define I2C_RXRDYEN			(0x08)
#define I2C_ARDYEN			(0x04)
#define I2C_NACKEN			(0x02)
#define I2C_ALEN			(0x01)

/* I2CSR 7.3 */
#define I2C_SDIR			(0x4000)
#define I2C_NACKSNT			(0x2000)
#define I2C_BB				(0x1000)
#define I2C_RSFULL			(0x0800)
#define I2C_XSMT			(0x0400)
#define I2C_AAS				(0x0200)
#define I2C_AD0				(0x0100)
#define I2C_SCD				(0x0020)
#define I2C_TXRDY			(0x0010)
#define I2C_RXRDY			(0x0008)
#define I2C_ARDY			(0x0004)
#define I2C_NACK			(0x0002)
#define I2C_AL				(0x0001)

/* I2CMDR 7.10 */
#define I2C_NACKMOD			(0x8000)
#define I2C_FREE			(0x4000)
#define I2C_STT				(0x2000)
#define I2C_LPM				(0x1000)
#define I2C_STP				(0x0800)
#define I2C_MST				(0x0400)
#define I2C_TRX				(0x0200)
#define I2C_XA				(0x0100)
#define I2C_RM				(0x0080)
#define I2C_DLB				(0x0040)
#define I2C_nIRS			(0x0020)
#define I2C_STB				(0x0010)
#define I2C_FDF				(0x0008)

/* I2CEMDR 7.12 */
#define I2C_BCM				(0x01)

/* I2CDIR 7.14 */
#define I2C_TXDMAEN			(0x20)
#define I2C_RXDMAEN			(0x10)
#define I2C_SDAFUNC			(0x08)
#define I2C_SCLFUNC			(0x04)
#define I2C_SDADIR			(0x02)
#define I2C_SCLDIR			(0x01)

/* I2CDOUT 7.15 */
#define I2C_SDAOUT			(0x02)
#define I2C_SCLOUT			(0x01)

/* I2CDIN 7.16 */
#define I2C_SDAIN 			(0x02)
#define I2C_SCLIN			(0x01)

/* I2CPFNC 7.17 */
#define I2C_PFUNC			(0x01)

/* I2CPDIR 7.18 */
#define I2C_SDADIR			(0x02)
#define I2C_SCLDIR			(0x01)

/* I2CDSET 7.21 */
#define I2C_SDASET 			(0x02)
#define I2C_SCLSET			(0x01)

/* I2CDCLR 7.22 */
#define I2C_SDACLR			(0x02)
#define I2C_OSCLIN			(0x01)

/*********************************** SPI *********************************/
/* SPICTRL2 */
#define SPI_CLKMOD      	(0x20)
#define SPI_SPIEN       	(0x10)
#define SPI_MASTER      	(0x08)
#define SPI_POWERDOWN   	(0x04)
#define SPI_POLARITY    	(0x02)
#define SPI_PHASE       	(0x01)

/* SPICTRL3 */
#define SPI_ENABLEHIGHZ 	(0x20)
#define SPI_DMAREQEN    	(0x10)
#define SPI_OVRNINTEN   	(0x08)
#define SPI_RCVROVRN    	(0x04)
#define SPI_RXINTEN     	(0x02)
#define SPI_RXINTFLAG   	(0x01)

/* SPIPC1 */
#define SPI_SCSDIR      	(0x10)
#define SPI_SOMIDIR     	(0x08)
#define SPI_SIMODIR     	(0x04)
#define SPI_CLKDIR      	(0x02)
#define SPI_ENADIR      	(0x01)

/* SPIPC2 */
#define SPI_SCSDIN      	(0x10)
#define SPI_SOMIDIN     	(0x08)
#define SPI_SIMODIN     	(0x04)
#define SPI_CLKDIN      	(0x02)
#define SPI_ENADIN      	(0x01)

/* SPIPC3 */
#define SPI_SCSDOUT     	(0x10)
#define SPI_SOMIDOUT    	(0x08)
#define SPI_SIMODOUT    	(0x04)
#define SPI_CLKDOUT     	(0x02)
#define SPI_ENADOUT     	(0x01)

/* SPIPC6 */
#define SPI_SCSFUN     		(0x10)
#define SPI_SOMIFUN     	(0x08)
#define SPI_SIMOFUN     	(0x04)
#define SPI_CLKFUN      	(0x02)
#define SPI_ENAFUN      	(0x01)

/*********************************** SCI *********************************/
/* SCICCR */
#define SCI_STOP        	(0x80)
#define SCI_PARITY      	(0x40)
#define SCI_PARITYENA   	(0x20)
#define SCI_TIMING_MODE 	(0x10)
#define SCI_COMM_MODE   	(0x08)
#define SCI_DATA_BITS(x)	(0x07 & (x-1))

/* SCICTL1 */
#define SCI_RXDMAALL    	(0x40)
#define SCI_RXDMAEN     	(0x20)
#define SCI_IDLE        	(0x10)
#define SCI_SLEEP       	(0x08)
#define SCI_RXRDY       	(0x04)
#define SCI_RXWAKE      	(0x02)
#define SCI_RXENA       	(0x01)

/* SCICTL2 */
#define SCI_CONT        	(0x80)
#define SCI_LOOPBACK    	(0x40)
#define SCI_TXDMAEN     	(0x20)
#define SCI_TXEMPTY     	(0x08)
#define SCI_TXRDY       	(0x04)
#define SCI_TXWAKE      	(0x02)
#define SCI_TXENA       	(0x01)

/* SCICTL3 */
#define SCI_SW_NRESET   	(0x80)
#define SCI_CLOCK       	(0x20)
#define SCI_RXACTIONENA		(0x10)
#define SCI_TXACTIONENA 	(0x08)

/* SCIRXST */
#define SCI_BUS_BUSY        (0x80)
#define SCI_FE			    (0x20)
#define SCI_OE			    (0x10)
#define SCI_PE			    (0x08)
#define SCI_WAKEUP			(0x04)
#define SCI_BRKDT			(0x02)
#define SCI_RXERR			(0x01)

/* SCIPC1 */
#define SCI_CLKDATAIN   	(0x08)
#define SCI_CLKDATAOUT  	(0x04)
#define SCI_CLKFUNC     	(0x02)
#define SCI_CLKDATADIR  	(0x01)

/* SCIPC2 */
#define SCI_RXDATAIN    	(0x08)
#define SCI_RXDATAOUT   	(0x04)
#define SCI_RXFUNC      	(0x02)
#define SCI_RXDATADIR   	(0x01)

/* SCIPC3 */
#define SCI_TXDATAIN    	(0x08)
#define SCI_TXDATAOUT  	 	(0x04)
#define SCI_TXFUNC     		(0x02)
#define SCI_TXDATADIR  		(0x01)

/*********************************** DMA *********************************/
/* DMAC */
#define DMA_INT_ENA 		(0x00008000)
#define DMA_INT_DIS			(0x00000000)
#define DMA_BITS_EIGHT 		(0x00000000)
#define DMA_BITS_SIXTEEN 	(0x00002000)
#define DMA_DEST_INC 		(0x00000800)
#define DMA_DEST_CNST 		(0x00000000)
#define DMA_SRC_INC 		(0x00000200)
#define DMA_SRC_CNST 		(0x00000000)
#define DMA_PERIPERIAL 		(0x0000000F)
#define DMA_MEM_SEL2 		(0x00000002)

/* DMACCx */
#define DMA_INT0	 		(0x00000000)
#define DMA_INT1 			(0x00000008)
#define DMA_SUSPEND_ENA		(0x00000000)
#define DMA_SUSPEND_DIS		(0x00000004)
#define DMA_REQ_DIS 		(0x00000000)
#define DMA_REQ_ENA 		(0x00000002)

/* DMAS */
#define DMA_TRANS_CMPLT		(0x00020000)
#define DMA_INT_OCCUR 		(0x00010000)

/* DMACCPx */
#define DMA_CHAN_ENA 		(0x00000040)

/********************************* SYSTEM ********************************/
/* CLKCTRL */
#define SYS_PPWNOVR			(0x00000080)
#define SYS_CLKSR_SYCLK		(0x00000060)
#define SYS_CLKSR_MCLK		(0x00000040)
#define SYS_CLKSR_ICLK		(0x00000020)
#define SYS_CLKDIR			(0x00000010)
#define SYS_CLKDOUT			(0x00000008)
#define SYS_CLKDIN			(0x00000004)
#define SYS_LPM_HALT		(0x00000003)
#define SYS_LPM_STBY		(0x00000002)
#define SYS_LPM_IDLE		(0x00000001)
#define SYS_LPM_RUN			(0x00000000)

/* GLBCTRL */
#define SYS_FLASH_ENA 		(0x00000010)
#define SYS_MULT_4			(0x00000008)
#define SYS_CLK_DIV_PRE(x)	((x) & 0x7)

/******************************* INTERRUPT *******************************/
/* RTICNTR */
#define INT_RTI_PRESCL 		(0x000007FF)
#define INT_RTI_CNTR		(0xFFFFF800)

/* RTIPCTL */
#define INT_RTI_PRELD 		(0x000007FF)

/* RTICINT */
#define INT_RTI_CMP1_FLG 	(0x00000080)
#define INT_RTI_CMP2_FLG 	(0x00000040)
#define INT_RTI_CMP1_ENA 	(0x00000020)
#define INT_RTI_CMP2_ENA 	(0x00000010)

/********************************** FLASH ********************************/
/* FMREGOPT */
#define FM32_ENA_PIPELINE 	(0x00000001)

/* FMBBUSY */
#define FM32_LVL2_UNLOCK 	(0x00004000)

/* FMBRDY */
#define FM16_BANK_READY		(0x0020)

/* FMMAC1 */
#define FM16_ENA_SECTR_CNTL (0x8000)

/* FMMSTAT */
#define FM16_ERASE_ACTIVE 	(0x00000080)
#define FM16_PROG_ACTIVE 	(0x00000040)
#define FM16_INVAL_DATA 	(0x00000020)
#define FM16_OPTN_FAILED 	(0x00000010)
#define FM16_3V_STAT 		(0x00000008)
#define FM16_LOCK_STATUS 	(0x00000001)

