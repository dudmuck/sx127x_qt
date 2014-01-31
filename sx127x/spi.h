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
#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "sx12xx-common_regs.h"
#include "sx127x-LoRa_v2a.h"
#include "sx12xx-Fsk.h"

#include <QRadioButton>
#include <QCheckBox>

#define SYS_MODE


#ifdef SYS_MODE // BCM numbers used in sys mode
    #define DIO0_PIN        17  // pin11
    #define DIO1_PIN        27  // pin13
    #define DIO2_PIN        22  // pin15
    #define DIO3_PIN        23  // pin16
    #define DIO4_PIN        24  // pin18
    #define DIO5_PIN        25  // pin22
    #define HWRESET_PIN     4   // pin7
    #define FEM_CPS_PIN     30  // sx1276 LF rf switch
    #define FEM_CTX_PIN     28  // sx1272 HF rf switch
#else
    #define DIO0_PIN        0   // pin11    GPIO0=BCM17
    #define DIO1_PIN        2   // pin13    GPIO2=BCM27(rv2)
    #define DIO2_PIN        3   // pin15    GPIO3=BCM22
    #define DIO3_PIN        4   // pin16    GPIO4=BCM23
    #define DIO4_PIN        5   // pin18    GPIO5=BCM24
    #define DIO5_PIN        6   // pin22    GPIO6=BCM25
    #define HWRESET_PIN     7   // pin7
    #define FEM_CPS_PIN     10  // sx1276 LF rf switch
    #define FEM_CTX_PIN     8   // sx1272 HF rf switch
#endif

typedef enum {
    RADIO_TYPE_NONE = 0,
    RADIO_TYPE_SX1272,
    RADIO_TYPE_SX1276
} radio_type_e;

typedef struct {
    uint8_t new_rx_pkt      : 1; // 0
    uint8_t flow_en         : 1; // 1
    uint8_t new_afc_value   : 1; // 2
    uint8_t busy_fifopull   : 1; // 3
    uint8_t stop_all        : 1; // 4
    uint8_t pn9_tx_test_    : 1; // 5
} flags_t;

typedef enum {
    NO_EDGE,
    FALLING_EDGE,
    RISING_EDGE
} edge_e;

extern flags_t flags;

#define LENGTH_UNKNOWN     0xffff
extern int rf_buf_len;
extern uint8_t rf_buf[2048];
extern unsigned int remaining_;
extern uint8_t NodeAdrs_to_send;

void radio_write(uint8_t addr, uint8_t val);
uint8_t radio_read(uint8_t addr);
void radio_fsk_write_fifo(uint8_t *pbuf, uint8_t len, bool first_part);
void radio_lora_write_fifo(uint8_t *pbuf, uint8_t len);
void reset_flow(void);
void radio_write_u16(uint8_t addr, uint16_t val);
uint8_t radio_read(uint8_t addr);
uint16_t radio_read_u16(uint8_t addr);

extern edge_e dio1_edge;
void set_dio1_edge(edge_e edge);
void set_bitrate(unsigned int bps);
void set_fdev(unsigned int hz);
void set_opmode( uint8_t opMode );
void set_rx_dcc_bw_hz(uint32_t bw_hz, bool afc);
uint32_t get_rx_bw_hz(uint8_t addr);
uint8_t get_pn9_byte(void);
extern int lfsr;
//void continue_pn9_tx_fifo(void);
void setMHz(float MHz);

extern bool sx1276;	// false = sx1272


extern uint32_t total_tx_pkts;
extern uint32_t pkt_count;
extern int pkt_tid;

extern QRadioButton *spi_radioButtonTX;
extern QRadioButton *spi_radioButtonStandby;
extern QRadioButton *spi_radioButtonSleep;
extern QRadioButton *spi_radioButtonFSTX;
extern QRadioButton *spi_radioButtonFSRX;
extern QRadioButton *spi_radioButtonRX;
extern QRadioButton *spi_radioButtonRxSingle;
extern QRadioButton *spi_radioButtonCAD;
void showOpMode(void);

extern QCheckBox *spi_checkBoxPayloadCrcError;
extern QCheckBox *spi_checkBoxTxDone;
extern QCheckBox *spi_checkBoxFHSS;
extern QCheckBox *spi_checkBoxRxTimeout;
extern QCheckBox *spi_checkBoxRxDone;
extern QCheckBox *spi_checkBoxValidHeader;
extern QCheckBox *spi_checkBoxCadDone;
extern QCheckBox *spi_checkBoxCadDetected;
void show_lora_irqflags(void);

extern uint32_t *xtal_hz;

extern char tx_busy;

void read_all_pins(uint8_t *pins);

class Spi
{
    public:
        Spi();
        //void RadioSetOpMode( uint8_t opMode );
        //void setMHz(float MHz);
        float getMHz(void);
        int spi_fd;
        radio_type_e type;
        void hw_reset(void);
        int open(void);

    private:


};

#endif // SPI_H
