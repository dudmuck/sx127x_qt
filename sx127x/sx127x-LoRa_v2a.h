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
#ifndef __SX1272_LORA_H__
#define __SX1272_LORA_H__

/*!
 * SX1272 v2a Internal registers Address
 * common registers in sx1272-common_regs.h 
 */
// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0d
#define REG_LR_FIFOTXBASEADDR                       0x0e
#define REG_LR_FIFORXBASEADDR                       0x0f
#define REG_LR_FIFORXCURRENTADDR					0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_NBRXBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUE_MSB                 0x14
#define REG_LR_RXHEADERCNTVALUE_LSB                 0x15
#define REG_LR_RXPACKETCNTVALUE_MSB                 0x16
#define REG_LR_RXPACKETCNTVALUE_LSB                 0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1a
#define REG_LR_RSSIVALUE                            0x1b
#define REG_LR_HOPCHANNEL                           0x1c
#define REG_LR_MODEMCONFIG1                         0x1d
#define REG_LR_MODEMCONFIG2                         0x1e
#define REG_LR_SYMBTIMEOUTLSB                       0x1f
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22 // and RX length for implicit
#define REG_LR_RX_MAX_PAYLOADLENGTH                 0x23 // length limit for explicit mode
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_RXBYTEADDR /*REG_LR_RXDATAADDR*/     0x25
#define REG_LR_PPM_CORRECTION_MSB                   0x26
#define REG_LR_PPM_CORRECTION_LSB                   0x27
#define REG_LR_TEST28                               0x28  // est_freq_error
#define REG_LR_TEST29                               0x29    // est_freq_error
#define REG_LR_TEST2A                               0x2a    // est_freq_error
#define REG_LR_TEST2B                               0x2b    // 
#define REG_LR_WIDEBAND_RSSI                        0x2c 
#define REG_LR_AGCH_TH                              0x2d    // agc_upper_th
#define REG_LR_AGCL_TH                              0x2e    // agc_lower_th
#define REG_LR_IFFRQH                               0x2f    // if_freq(12:8)
#define REG_LR_IFFRQL                               0x30    // if_freq(7:0)
#define REG_LR_TEST31                               0x31    // if_freq_auto, ...
#define REG_LR_TEST32                               0x32    // 
#define REG_LR_TEST33                               0x33
/***** registers above 0x40 are same as FSK/OOK page in sx1272-common_regs.h *****/

/*!
 * SX1272 LoRa bit control definition
 */

/*!
 * RegOpMode: lora-specific
 * modes 0-5 defined in sx1272-commom_regs.h
 */
#define RF_OPMODE_RECEIVER_SINGLE                 0x06
#define RF_OPMODE_CAD                             0x07

#if 0
#endif /* #if 0 */

typedef union {
    struct {    // sx127x register 0x12
        uint8_t CadDetected         : 1;    // 0
        uint8_t FhssChangeChannel   : 1;    // 1
        uint8_t CadDone             : 1;    // 2
        uint8_t TxDone              : 1;    // 3
        uint8_t ValidHeader         : 1;    // 4
        uint8_t PayloadCrcError     : 1;    // 5
        uint8_t RxDone              : 1;    // 6
        uint8_t RxTimeout           : 1;    // 7
    } bits;
    uint8_t octet;
} RegIrqFlags_t;

typedef union {
    struct {    // sx1272 register 0x18
        uint8_t detect			: 1;    // 0
        uint8_t sync			: 1;    // 1
        uint8_t rx_ongoing		: 1;    // 2
        uint8_t header_valid	: 1;    // 3
        uint8_t clear			: 1;    // 4
        uint8_t RxCodingRate	: 3;    // 5,6,7
    } bits;
    uint8_t octet;
} RegModemStatus_t;

typedef union {
    struct {    // sx1272 register 0x1c
        uint8_t FhssPresentChannel  : 6;    // 0,1,2,3,4,5
        uint8_t RxPayloadCrcOn		: 1;    // 6
        uint8_t PllTimeout          : 1;    // 7
    } bits;
    uint8_t octet;
} RegHopChannel_t;

typedef union {
    struct {    // sx1276 register 0x1d
        uint8_t ImplicitHeaderModeOn    : 1;    // 0
        uint8_t CodingRate              : 3;    // 1,2,3
        uint8_t Bw                      : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1d
        uint8_t LowDataRateOptimize		: 1;    // 0  (?ppm_offset?) For sf12/sf11 at 125KHz
        uint8_t RxPayloadCrcOn          : 1;    // 1
        uint8_t ImplicitHeaderModeOn    : 1;    // 2
        uint8_t CodingRate              : 3;    // 3,4,5
        uint8_t Bw                      : 2;    // 6,7
    } sx1272bits;
    uint8_t octet;
} RegModemConfig1_t;

//sx1276 where is        uint8_t ppm_offset              : 1;    // 0 LowDataRateOptimize
//sx1276 where is       uint8_t CrcOn                   : 1;    // 1

typedef union {
    struct {    // sx1276 register 0x1e
        uint16_t SymbTimeout             : 10;    // 0->7,  0,1
        uint16_t RxPayloadCrcOn          : 1;    // 2
        uint16_t TxContinuousMode        : 1;    // 3
        uint16_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1e
        uint16_t SymbTimeout             : 10;    // 0->7,  0,1
        uint16_t AgcAutoOn               : 1;    // 2
        uint16_t TxContinuousMode        : 1;    // 3
        uint16_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1272bits;
    uint16_t word;
} RegModemConfig2_timeout_t;


typedef union {
    struct {    // sx127x register 0x26
        uint8_t reserved	: 2;    // 0,1
        uint8_t AgcAutoOn	: 1;    // 2
        uint8_t LowDataRateOptimize: 1;    // 3   was "MobileNode"
        uint8_t unused		: 4;    // 4,5,6,7 
    } sx1276bits;
    uint8_t octet;
    uint8_t sx1272_ppm_correction_msb;
} RegModemConfig3_t;

typedef union {
    struct {    // sx1272 register 0x28
        uint8_t if_freq_msb              : 5;    // 0,1,2,3,4
        uint8_t payload_fine_timing_gain : 2;    // 5,6
        uint8_t override_dec_gain        : 1;    // 7 
    } bits;
    uint8_t octet;
} RegFreqIfMsb_t;


typedef union {
    struct {    // sx1232 register 0x31
        uint8_t detect_trig_same_peaks_nb   : 3;    // 0,1,2
        uint8_t disable_pll_time_out        : 1;    // 3
        uint8_t tracking_integral           : 2;    // 4,5
        uint8_t frame_synch_gain            : 1;    // 6 
        uint8_t if_freq_auto                : 1;    // 7 
    } bits;
    uint8_t octet;
} RegTest31_t;

/**********************************************************/

typedef struct
{
    // LoRa registers
    uint8_t RegFifoAddrPtr;                         // 0x0D
    uint8_t RegFifoTxBaseAddr;                      // 0x0E
    uint8_t RegFifoRxBaseAddr;                      // 0x0F
    uint8_t RegRxDataStartAddr;                     // 0x10
    uint8_t RegIrqFlagsMask;                        // 0x11
    RegIrqFlags_t RegIrqFlags;                      // 0x12
    uint8_t RegNbRxBytes;                           // 0x13
    uint8_t RxHeaderCntMsb;                         // 0x14
    uint8_t RxHeaderCntLsb;                         // 0x15
    uint16_t RxPacketCnt;                           // 0x16, 0x17
    RegModemStatus_t RegModemStatus;                // 0x18
    uint8_t RegPktSnrValue;                         // 0x19
    uint8_t RegPktRssiValue;                        // 0x1A
    uint8_t RegRssiValue;                           // 0x1B
    RegHopChannel_t RegHopChannel;					// 0x1C
    RegModemConfig1_t RegModemConfig1;              // 0x1D
    RegModemConfig2_timeout_t RegModemConfig2_timeout;              // 0x1e, 0x1f
    uint16_t RegPreambleLength;                     // 0x20->0x21
    uint8_t RegPayloadLength;                       // 0x22
    uint8_t RegRxMaxPayloadLength;                  // 0x23
    uint8_t RegHopPeriod;                           // 0x24
    uint8_t RegRxDataAddr;                          // 0x25
    RegModemConfig3_t RegModemConfig3;              // 0x26
    // Test
    uint8_t sx1272_ppm_correction_lsb;              // 0x27
    uint8_t test28;                                 // 0x28
    uint8_t test29;                                 // 0x29
    uint8_t test2a;                                 // 0x2a
    uint8_t test2b;                                 // 0x2b
    uint8_t test2c;                                 // 0x2c
    uint8_t test2d;                                 // 0x2d
    uint8_t test2e;                                 // 0x2e
    RegFreqIfMsb_t RegFreqIfMsb;                    // 0x2f
    uint8_t RegFreqIfLsb;                           // 0x30
    RegTest31_t RegTest31;                          // 0x31
    uint8_t test32;                                 // 0x32
    // Additional settings
    uint8_t test33;                                 // 0x33
    // Test
    uint8_t RegTestReserved34[0x40 - 0x34];         //
} tSX127xLoRa;	// sx1272v1 and sx1276

extern tSX127xLoRa* SX127xLoRa;

#if 0
#endif /* #if 0 */

#endif //__SX1272_LORA_H__
