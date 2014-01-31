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
#include <stdint.h> 

// Common settings
#define RF_FREQ_MHZ                                 868.3
#define TX_POWER                                    20        
#define PKT_CRC                                     1         // [0: OFF, 1: ON]

/**************************************************************************/
/* sx1272 registers common to both FSK and LoRa (basic radio functioning) */
/**************************************************************************/


/*!
 * SX1272 definitions
 */
#define XTAL_FREQ_DEFAULT                                   32000000

//#define FRF_MHZ		61.03515625e-6
//#define FRF_KHZ		61.03515625e-3
//#define FRF_HZ		61.03515625
/* 2^19 = 524288 */
#define FRF_KHZ			((cfg.xtal_hz/1e3) / (float)(524288))
#define FRF_HZ			(cfg.xtal_hz / (float)(524288))
#define FRF_MHZ			((cfg.xtal_hz/1e6) / (float)(524288))

#define FRF_MHZ_C			((*xtal_hz/1e6) / (float)(524288))
#define FRF_HZ_C			(*xtal_hz / (float)(524288))

/*!
 * SX1272 v2a Internal registers Address
 */
#define REG_FIFO                                 0x00
#define REG_OPMODE                               0x01
#define REG_FRFMSB                               0x06
#define REG_FRFMID                               0x07
#define REG_FRFLSB                               0x08
// common Tx settings
#define REG_PACONFIG                             0x09
#define REG_PARAMP                               0x0A
#define REG_OCP                                  0x0B 
// common Rx settings
#define REG_LNA                                     0x0C
// I/O settings
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
// Version
#define REG_VERSION                              0x42

// Version
#define REG_VERSION                                 0x42
// Additional settings
#define REG_AGCREF                                  0x43
#define REG_AGCTHRESH1                              0x44
#define REG_AGCTHRESH2                              0x45
#define REG_AGCTHRESH3                              0x46
#define REG_PLLHOP                                  0x4B
#define REG_TCXO                                    0x58
#define REG_PADAC                                   0x5A
#define REG_PLL                                     0x5C    // RX PLL bandwidth
#define REG_BITRATEFRAC_SX1276                      0x5d
#define REG_PLLLOWPN                                0x5E    // TX PLL bandwidth
#define REG_FORMERTEMP                              0x6C    // saved temp at last calibration
#define REG_BITRATEFRAC_SX1272                      0x70
#define REG_PLL_SX1276								0x70

/******************************************************************************/
/*!
 * RegFifo
 */

/*!
 * RegOpMode
 */
#define RF_OPMODE_SLEEP                           0x00
#define RF_OPMODE_STANDBY                         0x01  // Default
#define RF_OPMODE_SYNTHESIZER_TX                  0x02
#define RF_OPMODE_TRANSMITTER                     0x03
#define RF_OPMODE_SYNTHESIZER_RX                  0x04
#define RF_OPMODE_RECEIVER                        0x05
// opmodes 6,7 change fsk/lora.  defined in IP-specific header

/******************************************************************************/

typedef union {
    struct {    // sx127x register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t LowFrequencyModeOn_sx1276  : 1;    // 3		sx1276 only
        uint8_t unused   			: 2;    // 4,5
        uint8_t AccessSharedReg     : 1;    // 6  use FSK 0x0d->0x3f registers in Lora mode
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } bits;
    struct {    // sx127x register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t ModulationShaping_sx1272  : 2;    // 3,4
        uint8_t ModulationType      : 2;    // 5,6  FSK/OOK
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } fsk_bits;
    uint8_t octet;
} RegOpMode_t;

// RegFrf: 0x06, 0x07, 0x08
typedef union {
    struct {
        uint8_t frf_lsb;
        uint8_t frf_mid;
        uint8_t frf_msb;
        uint8_t reserved;
    } octets;
    uint32_t frf;
} RegFrf_t;

typedef union {
    struct {    // sx1272 register 0x09
        uint8_t OutputPower : 4;    // 0,1,2,3
        uint8_t MaxPower    : 3;    // 4,5,6		pmax_dBm = 10.8+(0.6*Maxpower)
        uint8_t PaSelect    : 1;    // 7        1=PA_BOOST	0=RFO
    } bits;
	/* OutputPower: 
     *     RFO: Pout_dBm = Pmax_dBm - (15-OutputPower)
     * PABOOST: Pout_dBm =       17 - (15-OutputPower)
     */
    uint8_t octet;
} RegPaConfig_t;

typedef union {
    struct {    // sx1272 register 0x0a
        uint8_t PaRamp				: 4;    // 0,1,2,3
        uint8_t LowPnTxPllOff		: 1;    // 4		sx1272 only
        uint8_t ModulationShaping_sx1276	: 2;    // 5,6
        uint8_t unused				: 1;    // 7
    } bits;
    uint8_t octet;
} RegPaRamp_t; //


typedef union {
    struct {    // sx127x register 0x0b
        uint8_t OcpTrim	: 5;    // 0,1,2,3,4
        uint8_t OcpOn	: 1;    // 5
        uint8_t unused	: 2;    // 6,7
    } bits;
    uint8_t octet;
} RegOcp_t; //


typedef union {
    struct {    // sx1272 register 0x0c
        uint8_t lnaBoostHF  : 2;    // 0,1
        uint8_t unused       : 1;    // 2
        uint8_t lnaBoostLF	: 2;    // 3,4  sx1276:lnaBoostLF	   sx1232?: trim_rx_crfo:add caps to RFo
        uint8_t rxfe_gain    : 3;    // 5,6,7        1=PA_BOOST
    } bits;
    uint8_t octet;
} RegLna_t; // RXFE

typedef union {
    struct {    // sx12xx register 0x40
        uint8_t Dio3Mapping     : 2;    // 0,1
        uint8_t Dio2Mapping     : 2;    // 2,3
        uint8_t Dio1Mapping     : 2;    // 4,5
        uint8_t Dio0Mapping     : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping1_t;

typedef union {
    struct {    // sx12xx register 0x41
        uint8_t MapPreambleDetect : 1;    // 0      //DIO4 assign: 1b=preambleDet 0b=rssiThresh
        uint8_t io_mode           : 3;    // 1,2,3  //0=normal,1=debug,2=fpga,3=pll_tx,4=pll_rx,5=analog
        uint8_t Dio5Mapping       : 2;    // 4,5
        uint8_t Dio4Mapping       : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping2_t;

typedef union {
    struct {    // sx12xx register 0x5a
        uint8_t prog_txdac      : 3;    // 0,1,2   4=5uA normal, 7=6.875uA for +20dBm
        uint8_t pds_analog_test : 1;    // 3
        uint8_t pds_pa_test     : 2;    // 4,5
        uint8_t trim_ptat       : 2;    // 6,7
    } bits;
    uint8_t octet;
} RegPaDac_t;

typedef union {
    struct {    // sx12xx register 0x5c
        uint8_t reserved     : 6;    // 0,1,2,3,4,5
        uint8_t pllBw       : 2;    // 6,7
    } bits;
    uint8_t octet;
} RegPll_t;

/******************************************************************************/


typedef struct
{
    uint8_t RegFifo;                                // 0x00
    // Common settings
    RegOpMode_t RegOpMode;                          // 0x01
    uint8_t FSK_RegBitrateMsb;                      // 0x02 unused in LoRa
    uint8_t FSK_RegBitrateLsb;                      // 0x03 unused in LoRa
    uint8_t FSK_RegFdevMsb;                         // 0x04 unused in LoRa
    uint8_t FSK_RegFdevLsb;                         // 0x05 unused in LoRa
    uint8_t RegFrfMsb;                              // 0x06
    uint8_t RegFrfMid;                              // 0x07
    uint8_t RegFrfLsb;                              // 0x08
    // Tx settings
    RegPaConfig_t RegPaConfig;                      // 0x09
    RegPaRamp_t RegPaRamp;                          // 0x0A
    RegOcp_t RegOcp;                                // 0x0B
    // Rx settings
    RegLna_t RegLna;                                // 0x0C
    uint8_t ip_regs[0x40-0x0d];                     // 0x0D->0x3F specific to FSK/OOK or LoRa 
    // I/O settings
    RegDioMapping1_t RegDioMapping1;                // 0x40
    RegDioMapping2_t RegDioMapping2;                // 0x41
    // Version
    uint8_t RegVersion;                             // 0x42
    // Additional settings
    uint8_t RegAgcRef;                              // 0x43
    uint8_t RegAgcThresh1;                          // 0x44
    uint8_t RegAgcThresh2;                          // 0x45
    uint8_t RegAgcThresh3;                          // 0x46
    // Test
    uint8_t RegTestReserved47[0x4B - 0x47];         // 0x47-0x4A
    // Additional settings
    uint8_t RegPllHop;                              // 0x4B
    // Test
    uint8_t RegTestReserved4C[0x58-0x4C];           // 0x4C-0x57
    // Additional settings
    uint8_t RegTcxo;                                // 0x58
    // Test
    uint8_t RegTestReserved59;                      // 0x59
    // Additional settings
    RegPaDac_t RegPaDac;                             // 0x5A
    // Test
    uint8_t RegTestReserved5B;                      // 0x5B
    // Additional settings
    RegPll_t RegPll;                                // 0x5C
    // Test
    uint8_t RegBitRateFrac_sx1276;                  // 0x5D
    // Additional settings
    RegPll_t RegPllLowPn;                            // 0x5E
    // Test
	uint8_t Reg5f;
	uint8_t Reg60;
	uint8_t Reg61;
	uint8_t Reg62;
	uint8_t Reg63;
	uint8_t Reg64;
	uint8_t Reg65;
	uint8_t Reg66;
	uint8_t Reg67;
	uint8_t Reg68;
	uint8_t Reg69;
	uint8_t Reg6a;
	uint8_t Reg6b;
    // Additional settings
    uint8_t RegFormerTemp;                          // 0x6C
    // Test
	uint8_t Reg6d;									// 0x6d
	uint8_t Reg6e;									// 0x6e
	uint8_t Reg6f;									// 0x6f
	RegPll_t RegPll_sx1276;							// 0x70
} tSX127x;

extern tSX127x SX127x;
