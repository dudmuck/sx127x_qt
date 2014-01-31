/*
 *  This file is part of sx127x_qt.
 *
 *  sx127x_qt is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  sx127x_qt is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with sx127x_qt.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __SX1272_FSK_H__
#define __SX1272_FSK_H__


/*!
 * SX1272 FSK General parameters definition
 */

#define FSK_FIFO_SIZE		64
#define FSK_FIFO_SIZE_HALF	(FSK_FIFO_SIZE>>1)

/*!
 * SX1272 Internal registers Address
 * common registers in sx1272-common_regs.h 
 */
#define REG_FSK_BITRATEMSB			0x02
#define REG_FSK_BITRATELSB			0x03
#define REG_FSK_FDEVMSB				0x04 
#define REG_FSK_FDEVLSB				0x05
#define REG_FSK_FRFMSB				0x06
#define REG_FSK_FRFMID				0x07
#define REG_FSK_FRFLSB				0x08
// Rx settings
#define REG_FSK_RXCONFIG			0x0D
#define REG_FSK_RSSICONFIG			0x0E
#define REG_FSK_RSSICOLLISION		0x0F	// rssi delta threshold (interferer)
#define REG_FSK_RSSITHRESH			0x10	// trigger level for rssi interrupt
#define REG_FSK_RSSIVALUE			0x11
#define REG_FSK_RXBW				0x12 
#define REG_FSK_AFCBW				0x13
#define REG_FSK_OOKPEAK				0x14	// bitsync config
#define REG_FSK_OOKFIX				0x15	// threshold dB
#define REG_FSK_OOKAVG				0x16	
#define REG_FSK_RES17				0x17	// barker test
#define REG_FSK_RES18				0x18	// barker test
#define REG_FSK_RES19				0x19	// barker test
#define REG_FSK_AFCFEI				0x1A
#define REG_FSK_AFCMSB				0x1B
#define REG_FSK_AFCLSB				0x1C
#define REG_FSK_FEIMSB				0x1D
#define REG_FSK_FEILSB				0x1E
#define REG_FSK_PREAMBLEDETECT		0x1F
#define REG_FSK_RXTIMEOUT1			0x20	// rssi timeout
#define REG_FSK_RXTIMEOUT2			0x21	// preamble detect timeout
#define REG_FSK_RXTIMEOUT3			0x22	// sync detect timeout
#define REG_FSK_RXDELAY				0x23	// RX restart delay
// Oscillator settings
#define REG_FSK_OSC					0x24	// clkout output divider
// Packet handler settings
#define REG_FSK_PREAMBLEMSB			0x25	// preamble length
#define REG_FSK_PREAMBLELSB			0x26	// preamble length
#define REG_FSK_SYNCCONFIG			0x27
#define REG_FSK_SYNCVALUE1			0x28
#define REG_FSK_SYNCVALUE2			0x29
#define REG_FSK_SYNCVALUE3			0x2A
#define REG_FSK_SYNCVALUE4			0x2B
#define REG_FSK_SYNCVALUE5			0x2C
#define REG_FSK_SYNCVALUE6			0x2D
#define REG_FSK_SYNCVALUE7			0x2E
#define REG_FSK_SYNCVALUE8			0x2F
#define REG_FSK_PACKETCONFIG1		0x30
#define REG_FSK_PACKETCONFIG2		0x31
#define REG_FSK_PAYLOADLENGTH		0x32
#define REG_FSK_NODEADRS			0x33
#define REG_FSK_BROADCASTADRS		0x34
#define REG_FSK_FIFOTHRESH			0x35
// SM settings
#define REG_FSK_SEQCONFIG1			0x36
#define REG_FSK_SEQCONFIG2			0x37
#define REG_FSK_TIMERRESOL			0x38
#define REG_FSK_TIMER1COEF			0x39	// period of timer1 interrupt
#define REG_FSK_TIMER2COEF			0x3A	// period of timer2 interrupt
// Service settings
#define REG_FSK_IMAGECAL			0x3B
#define REG_FSK_TEMP				0x3C
#define REG_FSK_LOWBAT				0x3D	// EOL "end of life"
// Status
#define REG_FSK_IRQFLAGS1			0x3E
#define REG_FSK_IRQFLAGS2			0x3F	// packet flags
/***** registers above 0x40 are same as FSK/OOK page in sx1272-common_regs.h *****/

/*!
 * SX1272 FSK bit control definition
 */

/*!
 * RegFifo
 */

#if 0
/*!
 * RegIrqFlags1
 */
#define RF_IRQFLAGS1_MODEREADY					  0x80

#define RF_IRQFLAGS1_RXREADY						0x40

#define RF_IRQFLAGS1_TXREADY						0x20

#define RF_IRQFLAGS1_PLLLOCK						0x10

#define RF_IRQFLAGS1_RSSI						   0x08

#define RF_IRQFLAGS1_TIMEOUT						0x04

#define RF_IRQFLAGS1_PREAMBLEDETECT				 0x02

#define RF_IRQFLAGS1_SYNCADDRESSMATCH			   0x01

/*!
 * RegIrqFlags2
 */
#define RF_IRQFLAGS2_FIFOFULL					   0x80

#define RF_IRQFLAGS2_FIFOEMPTY					  0x40

#define RF_IRQFLAGS2_FIFOLEVEL					  0x20

#define RF_IRQFLAGS2_FIFOOVERRUN					0x10

#define RF_IRQFLAGS2_PACKETSENT					 0x08

#define RF_IRQFLAGS2_PAYLOADREADY				   0x04

#define RF_IRQFLAGS2_CRCOK						  0x02

#define RF_IRQFLAGS2_LOWBAT						 0x01
#endif /* #if 0 */

typedef union {
	struct {	// sx1272 register 0x0d
		//uint8_t wait_rssi_irq		   : 1;	// 0 wait for signal strength before entering RX
		//uint8_t wait_irq_0x55		   : 1;	// 1 wait for preamble before entering RX
		//uint8_t agc_on_irq_0x55		 : 1;	// 2 1=LNA gain adj done until irq_0x55 asserted
		uint8_t RxTrigger			   : 3;	// 0,1,2: 0=none 1=rssiInt 6=preambleDet 7=both
		uint8_t AgcAutoOn			   : 1;	// 3
		uint8_t AfcAutoOn			   : 1;	// 4
		uint8_t RestartRxWithPllLock	: 1;	// 5		restart from FSRX mode
		uint8_t RestartRxWithoutPllLock : 1;	// 6
		uint8_t RestartRxOnCollision	: 1;	// 7
	} bits;
	uint8_t octet;
} RegRxConfig_t;

typedef union {
	struct {	// sx1272 register 0x0e
		uint8_t RssiSmoothing	: 3;	// 0,1,2
		uint8_t RssiOffset		: 5;	// 3,4,5,6,7
	} bits;
	uint8_t octet;
} RegRssiConfig_t;


typedef union {
	struct {	// sx1272 register 0x12
		//uint8_t RxBw			: 5;	// 0,1,2,3,4		(0,1,2=exp   3,4=mant)
		uint8_t Exponent		: 3;	// 0,1,2
		uint8_t Mantissa		: 2;	// 3,4
		uint8_t dcc_force	   : 1;	// 5 force dcc on all rxbw (otherwise put only if > 167KHz)
		uint8_t dcc_fast_init   : 1;	// 6 
		uint8_t reserved		: 1;	// 7 
	} bits;
	uint8_t octet;
} RegRxBw_t;

typedef union {
	struct {	// sx1272 register 0x14
		uint8_t OokPeakThreshStep   : 3;	// 0,1,2
		uint8_t OokThreshType	   : 2;	// 3,4
		uint8_t BitSyncOn		   : 1;	// 5
		uint8_t barker_en		   : 1;	// 6
		uint8_t bsync_opt		   : 1;	// 7	not used
	} bits;
	uint8_t octet;
} RegOokPeak_t; // DEMOD1 0x14

typedef union {
	struct {	// sx1272 register 0x1a
		uint8_t AfcAutoClearOn  : 1;	// 0
		uint8_t AfcClear		: 1;	// 1	manual clear
		uint8_t unused1		 : 1;	// 2
		uint8_t fei_range	   : 1;	// 3	FEI range limited by: 0=rxbw	1=fs/2
		uint8_t AgcStart		: 1;	// 4	manual trigger AGC
		uint8_t unused		  : 3;	// 5,6,7 
	} bits;
	uint8_t octet;
} RegAfcFei_t;

typedef union {
	struct {	// sx1272 register 0x1f
		uint8_t PreambleDetectorTol	 : 5;	// 0,1,2,3,4	allowed chip errors
		uint8_t PreambleDetectorSize	: 2;	// 5,6	  00b=1bytes... 11b=4bytes
		uint8_t PreambleDetectorOn	  : 1;	// 7
	} bits;
	uint8_t octet;
} RegPreambleDetect_t;

typedef union {
	struct {	// sx1232 register 0x27
		uint8_t SyncSize			: 3;	// 0,1,2
		uint8_t FifoFillCondition   : 1;	// 3	rx fifo fill starting 0=start-on-sync
		uint8_t SyncOn			  : 1;	// 4	enable pattern recognition
		uint8_t PreamblePolarity	: 1;	// 5	0=0xaa 1=0x55
		uint8_t AutoRestartRxMode   : 2;	// 6,7  00b=do not restart 10b=wait-for-pll
	} bits;
	uint8_t octet;
} RegSyncConfig_t;

typedef union {
	struct {	// sx1232 register 0x30
		uint8_t CrCWhiteningType : 1;	// 0	   1=IBM-crc   0=ccitt-crc
		uint8_t AddressFiltering : 2;	// 1,2	 11b = two-byte nodeadrs at 0x2c->0x2f
		uint8_t CrcAutoClearOff  : 1;	// 3
		uint8_t CrcOn			: 1;	// 4
		uint8_t DcFree		   : 2;	// 5,6 
		uint8_t PacketFormatVariable : 1;	// 7	   1=variable length, 0=fixed
	} bits;
	uint8_t octet;
} RegPktConfig1_t;

typedef union {
	struct {	// sx1272 register 0x31 and 0x32
		uint16_t PayloadLength		: 11;	// 0->10
		uint16_t BeaconOn			: 1;	// 11 
		uint16_t IoHomePowerFrame	: 1;	// 12   CRC LFSR init: 0=0x1d0f, 1=0x0000=powerlink
		uint16_t IoHomeOn			: 1;	// 13
		uint16_t DataModePacket		: 1;	// 14   1=packet mode, 0=continuous mode
		uint16_t unused				: 1;	// 15 
	} bits;
	uint16_t word;
} RegPktConfig2_t;

typedef union {
	struct {	// sx1272 register 0x35
		uint8_t FifoThreshold	   : 6;	// 0,1,2,3,4,5
		uint8_t unused			  : 1;	// 6 
		uint8_t TxStartCondition	: 1;	// 7		0=fifoThresh 1=fifoNotEmpty
	} bits;
	uint8_t octet;
} RegFifoThreshold_t;

typedef union {
	struct {	// sx1272 register 0x36
		uint8_t FromTransmit		: 1;	// 0
		uint8_t FromIdle			: 1;	// 1
		uint8_t LowPowerSelection   : 1;	// 2
		uint8_t FromStart		   : 2;	// 3,4
		uint8_t IdleMode			: 1;	// 5 
		uint8_t SequencerStop	   : 1;	// 6 
		uint8_t SequencerStart	  : 1;	// 7
	} bits;
	uint8_t octet;
} RegSeqConfig1_t;   // @0x36

typedef union {
	struct {	// sx1272 register 0x37
		uint8_t FromPacketReceived  : 3;	// 0,1,2
		uint8_t FromRxTimeout	   : 2;	// 3,4
		uint8_t FromReceive		 : 3;	// 5,6,7
	} bits;
	uint8_t octet;
} RegSeqConfig2_t;   // @0x37

typedef union {
	struct {	// sx1272 register 0x38
		uint8_t timer2_resol   : 2;	// 0,1
		uint8_t timer1_resol   : 2;	// 2,3
		uint8_t force_hlm_irq  : 1;	// 4 
		uint8_t hlm_started	: 1;	// 5 
		uint8_t unused		 : 2;	// 6,7
	} bits;
	uint8_t octet;
} RegTimerResol_t;   // HL42 @0x38

typedef union {
	struct {	// sx1272 register 0x3b
		uint8_t TempMonitorOff	: 1;	// 0
		uint8_t TempThreshold	: 2;	// 1,2
		uint8_t TempChange		: 1;	// 3	read-only
		uint8_t unused			: 1;	// 4 
		uint8_t ImageCalRunning	: 1;	// 5	read-only
		uint8_t ImageCalStart	: 1;	// 6	write-only
		uint8_t AutoImageCalOn	: 1;	// 7
	} bits;
	uint8_t octet;
} RegImageCal_t;   // 

typedef union {
	struct {	// sx127x register 0x3d
		uint8_t LowBatTrim		: 3;	// 0,1,2
		uint8_t LowBatOn		: 1;	// 3	read-only
		uint8_t unused			: 4;	// 4,5,6,7
	} bits;
	uint8_t octet;
} RegLowBat_t ;   // 


typedef union {
	struct {	// sx1232 register 0x3e
		uint8_t SyncAddressMatch	: 1;	// 0 
		uint8_t PreambleDetect	  : 1;	// 1 
		uint8_t Timeout			 : 1;	// 2	rx-timeout
		uint8_t Rssi				: 1;	// 3 
		uint8_t PllLock			 : 1;	// 4 
		uint8_t TxReady			 : 1;	// 5 
		uint8_t RxReady			 : 1;	// 6 
		uint8_t ModeReady		   : 1;	// 7 
	} bits;
	uint8_t octet;
} RegIrqFlags1_t;   // STAT0

typedef union {
	struct {	// sx1232 register 0x3f
		uint8_t LowBat		  : 1;	// 0	"eol"
		uint8_t CrcOk		   : 1;	// 1 
		uint8_t PayloadReady	: 1;	// 2 
		uint8_t PacketSent	  : 1;	// 3 
		uint8_t FifoOverrun	 : 1;	// 4 
		uint8_t FifoLevel	   : 1;	// 5 
		uint8_t FifoEmpty	   : 1;	// 6 
		uint8_t FifoFull		: 1;	// 7 
	} bits;
	uint8_t octet;
} RegIrqFlags2_t;   // STAT1 @0x3f


/*typedef union {
	struct {	// sx1232 register 0x41
		uint8_t dagc_gain	   : 4;	// 0,1,2,3
		uint8_t dagc_setgain	: 1;	// 4  0=auto  1=from dagc_gain
		uint8_t rx_invert	   : 1;	// 5
		uint8_t conjug_iq	   : 1;	// 6	1: I-jQ	 0: I+jQ
		uint8_t unused		  : 1;	// 7 
	} bits;
	uint8_t octet;
} RegDagcTest_t;*/

typedef struct __attribute__((__packed__)) // registers specific to FSK/OOK IP
{
	RegRxConfig_t RegRxConfig;							// 0x0D
	RegRssiConfig_t RegRssiConfig;						  // 0x0E
	uint8_t RegRssiCollision;					   // 0x0F
	uint8_t RegRssiThresh;						  // 0x10
	uint8_t RegRssiValue;						   // 0x11
	RegRxBw_t RegRxBw;							  // 0x12
	RegRxBw_t RegAfcBw;							 // 0x13
	RegOokPeak_t RegOokPeak;						// 0x14
	uint8_t RegOokFix;							  // 0x15
	uint8_t RegOokAvg;							  // 0x16
	uint8_t RegRes17;							   // 0x17
	uint8_t RegRes18;							   // 0x18
	uint8_t RegRes19;							   // 0x19
	RegAfcFei_t RegAfcFei;						  // 0x1A
    int16_t RegAfc;							  // 0x1b->0x1c
    int16_t RegFei;							  // 0x1d->0x1e
	RegPreambleDetect_t RegPreambleDetect;		  // 0x1F
	uint8_t RegRxTimeout1;						  // 0x20
	uint8_t RegRxTimeout2;						  // 0x21
	uint8_t RegRxTimeout3;						  // 0x22
	uint8_t RegRxDelay;							 // 0x23
	// Oscillator settings
	uint8_t RegOsc;								 // 0x24
	// Packet handler settings
    uint16_t RegPreambleSize;						 // 0x25->0x26
	RegSyncConfig_t RegSyncConfig;				  // 0x27
    uint8_t RegSyncValues[8];						  // 0x28->0x2f
	RegPktConfig1_t RegPktConfig1;			// 0x30
	RegPktConfig2_t RegPktConfig2;			// 0x31, 0x32	includes entire PayloadLength
	uint8_t RegNodeAdrs;							// 0x33
	uint8_t RegBroadcastAdrs;					   // 0x34
	RegFifoThreshold_t RegFifoThreshold;			// 0x35
	// Sequencer settings
	RegSeqConfig1_t RegSeqConfig1;				  // 0x36
	RegSeqConfig2_t RegSeqConfig2;				  // 0x37
	RegTimerResol_t RegTimerResol;				  // 0x38
	uint8_t RegTimer1Coef;						  // 0x39
	uint8_t RegTimer2Coef;						  // 0x3A
	// Service settings
	RegImageCal_t RegImageCal;					  // 0x3B
	uint8_t RegTemp;								// 0x3C
	RegLowBat_t RegLowBat;							  // 0x3D
	// Status
	RegIrqFlags1_t RegIrqFlags1;					// 0x3E
	RegIrqFlags2_t RegIrqFlags2;					// 0x3F
} tSX127xFSK;

extern tSX127xFSK* SX127xFSK;

#endif //__SX1272_FSK_H__

