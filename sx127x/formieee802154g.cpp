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
#include "formieee802154g.h"
#include "ui_formieee802154g.h"
#include <stdio.h>
#include "spi.h"
#include <wiringPi.h>
#include <math.h>


#define SFD_FEC			0x6f4e
#define SFD_UNCODED		0x904e

#define POLY_ITUT 		0x1021
#define INITIAL_CRC16	0x0000

#define INITIAL_CRC32	0xffffffff

/* because IEEE sends LSB first in time,
 * all struct bit fields are revsered bit-order because they are sent thru radio packet engine */

#define PHR_LENGTH	2	// two octets
typedef union {
    struct {
        uint16_t frame_length   : 11; // 10->0
        uint16_t DW             :  1; // 11
        uint16_t FCS            :  1; // 12         1=16bitCRC, 0=32bitCRC
        uint16_t reserved       :  2; // 14,13
        uint16_t MS             :  1; // 15         always 0 for non-mode-switch
    } bits;
    uint16_t word;
} MRFSK_PHR_t;  // without mode switching


MRFSK_PHR_t phr;

FormIEEE802154g::FormIEEE802154g(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FormIEEE802154g)
{
    ui->setupUi(this);
    tx_cnt = 0;
    payload_octet = -1;
    MFR_length = 4;	// FCS checkbox defaults to uncheck = 4byte crc
    psdu_buf_ = &phr_psdu_buf[2];	// PHR is first two bytes, PSDU is the rest
    crc_good_count = 0;

    ui->spinBoxWrongBit->setEnabled(false);
    ui->labelWrongSFD->setEnabled(false);

    ui->labelTXinterval->setEnabled(false);
    ui->labelTXcount->setEnabled(false);
    ui->spinBoxTXinterval->setEnabled(false);
    ui->spinBoxTXcount->setEnabled(false);

    ui->pushButtonTXPN9->setEnabled(false);
}

FormIEEE802154g::~FormIEEE802154g()
{
    delete ui;
}

void FormIEEE802154g::on_comboBoxInit_currentIndexChanged(int index)
{
    flags.stop_all = 1;

    set_opmode(RF_OPMODE_STANDBY);

    ui->comboBoxChannel->clear();

    /* FAN preamble length:
     * 50kbps or 100kbps: 8bytes
     * 150kbps: 12bytes
     * 200kbps: 20bytes
     * 300kbps: 24bytes
     */

    switch (index) {
        case 0: break;
        case 9:	// 50kbps, h=1
        case 1:	// 50kbps, h=1
            if (index == 1) {
                ui->comboBoxChannel->addItem("1: 920.6MHz", QVariant(920.6));
                ui->comboBoxChannel->addItem("2: 921.8MHz", QVariant(921.8));
                ui->comboBoxChannel->addItem("3: 923.0MHz", QVariant(923.0));
                ui->comboBoxChannel->addItem("4: 924.2MHz", QVariant(924.2));
            } else if (index == 9) {
                ui->comboBoxChannel->addItem("1: 870.2MHz", QVariant(870.2));
                ui->comboBoxChannel->addItem("2: 871.2MHz", QVariant(871.2));
                ui->comboBoxChannel->addItem("3: 872.2MHz", QVariant(872.2));
                ui->comboBoxChannel->addItem("4: 874.2MHz", QVariant(874.2));
            }
            set_bitrate(50000);
            set_fdev(25000);
            set_rx_dcc_bw_hz(62500, false);
            set_rx_dcc_bw_hz(62500, true);
            SX127xFSK->RegPreambleSize = 8;
            break;
        case 2:	// 100kbps, h=1
            ui->comboBoxChannel->addItem("1: 920.9MHz", QVariant(920.9));
            ui->comboBoxChannel->addItem("2: 922.1MHz", QVariant(922.1));
            ui->comboBoxChannel->addItem("3: 923.3MHz", QVariant(923.3));
            ui->comboBoxChannel->addItem("4: 924.5MHz", QVariant(924.5));
            set_bitrate(100000);
            set_fdev(50000);
            set_rx_dcc_bw_hz(125000, false);
            set_rx_dcc_bw_hz(125000, true);
            SX127xFSK->RegPreambleSize = 8;
            break;
        case 3:	// 200kbps, h=1
            ui->comboBoxChannel->addItem("1: 920.8MHz", QVariant(920.8));
            ui->comboBoxChannel->addItem("2: 921.4MHz", QVariant(921.4));
            ui->comboBoxChannel->addItem("3: 923.2MHz", QVariant(923.2));
            ui->comboBoxChannel->addItem("4: 925.6MHz", QVariant(925.6));
            set_bitrate(200000);
            set_fdev(100000);
            set_rx_dcc_bw_hz(250000, false);
            set_rx_dcc_bw_hz(250000, true);
            SX127xFSK->RegPreambleSize = 20;
            break;
        case 13:	// 300kbps, h=0.5
        case 8:		// 300kbps, h=0.5
        case 4:		// 300kbps, h=0.5
            if (index == 4) {
                ui->comboBoxChannel->addItem("1: 920.8MHz", QVariant(920.8));
                ui->comboBoxChannel->addItem("2: 921.4MHz", QVariant(921.4));
                ui->comboBoxChannel->addItem("3: 923.2MHz", QVariant(923.2));
                ui->comboBoxChannel->addItem("4: 925.6MHz", QVariant(925.6));
            } else if (index == 8) {
                ui->comboBoxChannel->addItem("1: 902.6MHz", QVariant(902.6));
                ui->comboBoxChannel->addItem("2: 905.9MHz", QVariant(905.9));
                ui->comboBoxChannel->addItem("3: 917.6MHz", QVariant(917.6));
                ui->comboBoxChannel->addItem("4: 926.6MHz", QVariant(926.6));
            } else if (index == 13) {
                ui->comboBoxChannel->addItem("1: 870.6MHz", QVariant(870.6));
                ui->comboBoxChannel->addItem("2: 871.6MHz", QVariant(871.6));
                ui->comboBoxChannel->addItem("3: 873.0MHz", QVariant(873.0));
                ui->comboBoxChannel->addItem("4: 874.2MHz", QVariant(874.2));
            }
            set_bitrate(300000);
            set_fdev(75000);
            set_rx_dcc_bw_hz(250000, false);
            set_rx_dcc_bw_hz(250000, true);
            SX127xFSK->RegPreambleSize = 24;
            break;
        case 10:	// 100kbps, h=0.5
        case 5:		// 100kbps, h=0.5
            if (index == 5) {
                ui->comboBoxChannel->addItem("1: 902.2MHz", QVariant(902.2));
                ui->comboBoxChannel->addItem("2: 904.2MHz", QVariant(904.2));
                ui->comboBoxChannel->addItem("3: 918.2MHz", QVariant(918.2));
                ui->comboBoxChannel->addItem("4: 926.2MHz", QVariant(926.2));
            } else if (index == 10) {
                ui->comboBoxChannel->addItem("1: 870.2MHz", QVariant(870.2));
                ui->comboBoxChannel->addItem("2: 871.2MHz", QVariant(871.2));
                ui->comboBoxChannel->addItem("3: 872.2MHz", QVariant(872.2));
                ui->comboBoxChannel->addItem("4: 874.2MHz", QVariant(874.2));
            }
            set_bitrate(100000);
            set_fdev(25000);
            set_rx_dcc_bw_hz(100000, false);
            set_rx_dcc_bw_hz(100000, true);
            SX127xFSK->RegPreambleSize = 24;
            break;
        case 11:	// 150kbps, h=0.5
        case 6:		// 150kbps, h=0.5
            if (index == 6) {
                ui->comboBoxChannel->addItem("1: 902.6MHz", QVariant(902.4));
                ui->comboBoxChannel->addItem("2: 904.4MHz", QVariant(904.4));
                ui->comboBoxChannel->addItem("3: 918.4MHz", QVariant(918.4));
                ui->comboBoxChannel->addItem("4: 926.4MHz", QVariant(926.4));
            } else if (index == 11) {
                ui->comboBoxChannel->addItem("1: 870.4MHz", QVariant(870.4));
                ui->comboBoxChannel->addItem("2: 871.4MHz", QVariant(871.4));
                ui->comboBoxChannel->addItem("3: 872.4MHz", QVariant(872.4));
                ui->comboBoxChannel->addItem("4: 874.4MHz", QVariant(874.4));
            }
            set_bitrate(150000);
            set_fdev(37500);
            set_rx_dcc_bw_hz(125000, false);
            set_rx_dcc_bw_hz(125000, true);
            SX127xFSK->RegPreambleSize = 24;
            break;
        case 12:	// 200kbps, h=0.5
        case 7:		// 200kbps, h=0.5
            if (index == 7) {
                ui->comboBoxChannel->addItem("1: 902.4MHz", QVariant(902.4));
                ui->comboBoxChannel->addItem("2: 904.4MHz", QVariant(904.4));
                ui->comboBoxChannel->addItem("3: 918.4MHz", QVariant(918.4));
                ui->comboBoxChannel->addItem("4: 926.4MHz", QVariant(926.4));
            } else if (index == 12) {
                ui->comboBoxChannel->addItem("1: 870.4MHz", QVariant(870.4));
                ui->comboBoxChannel->addItem("2: 871.4MHz", QVariant(871.4));
                ui->comboBoxChannel->addItem("3: 872.4MHz", QVariant(872.4));
                ui->comboBoxChannel->addItem("4: 874.4MHz", QVariant(874.4));
            }
            set_bitrate(200000);
            set_fdev(50000);
            set_rx_dcc_bw_hz(200000, false);
            set_rx_dcc_bw_hz(200000, true);
            SX127xFSK->RegPreambleSize = 24;
            break;
    }

    // -1 because first byte of sync will be last byte of preamble
    radio_write_u16(REG_FSK_PREAMBLEMSB, SX127xFSK->RegPreambleSize - 1);

    SX127xFSK->RegPktConfig2.bits.DataModePacket = 1; // packet mode
    radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

    SX127xFSK->RegPktConfig1.bits.PacketFormatVariable = 0; // fixed-length format
    SX127xFSK->RegPktConfig1.bits.CrcOn = 0;    // ieee CRCs handled by software
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);

    if (SX127x.RegDioMapping2.bits.Dio5Mapping != 2) { // to Data (for observation)
        SX127x.RegDioMapping2.bits.Dio5Mapping = 2;
        radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
    }

    /* to PayloadReady / PacketSent */
    if (SX127x.RegDioMapping1.bits.Dio0Mapping != 0 || SX127x.RegDioMapping1.bits.Dio2Mapping != 3) {
        SX127x.RegDioMapping1.bits.Dio0Mapping = 0;
        SX127x.RegDioMapping1.bits.Dio2Mapping = 3;	// for SyncAddress observation
        radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
    }

    SX127xFSK->RegSyncConfig.bits.PreamblePolarity = 1; // to 0x55
    SX127xFSK->RegSyncConfig.bits.SyncSize = 2;	// 2: one byte preamble + two byte SFD
    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);

    set_sfd(ui->checkBoxFECenable->isChecked());

    // start tx on !FifoEmpty, for small packets
    SX127xFSK->RegFifoThreshold.bits.TxStartCondition = 1;
    radio_write(REG_FSK_FIFOTHRESH, SX127xFSK->RegFifoThreshold.octet);

    SX127x.RegPaConfig.bits.PaSelect = 1;	// PA_BOOST pin
    radio_write(REG_PACONFIG, SX127x.RegPaConfig.octet);
}


uint8_t FormIEEE802154g::reverse_octet(uint8_t octet)
{
    uint8_t ret = (octet & 0x80) >> 7;
    ret |= (octet & 0x40) >> 5;
    ret |= (octet & 0x20) >> 3;
    ret |= (octet & 0x10) >> 1;
    ret |= (octet & 0x08) << 1;
    ret |= (octet & 0x04) << 3;
    ret |= (octet & 0x02) << 5;
    ret |= (octet & 0x01) << 7;
    return ret;
}

void FormIEEE802154g::interleave(uint8_t *buf)
{
    uint8_t out[4];
    fprintf(stderr, " in %02x %02x %02x %02x ->",
        buf[0],
        buf[1],
        buf[2],
        buf[3]
    );

    // 00112233 44556677 8899aabb ccddeeff	src
    // ffbb7733 eeaa6622 dd995511 cc884400	dst
    out[0] = (buf[3] & 0x03) << 6;	// ff to 00
    out[0] |= (buf[2] & 0x03) << 4;	// bb to 11
    out[0] |= (buf[1] & 0x03) << 2;	// 77 to 22
    out[0] |= buf[0] & 0x03;		// 33 to 33

    out[1] = (buf[3] & 0x0c) << 4;	// ee to 44
    out[1] |= (buf[2] & 0x0c) << 2;	// aa to 55
    out[1] |= buf[1] & 0x0c;		// 66 to 66
    out[1] |= (buf[0] & 0x0c) >> 2;	// 22 to 77

    out[2] = (buf[3] & 0x30) << 2;// dd to 88
    out[2] |= buf[2] & 0x30;// 99 to 99
    out[2] |= (buf[1] & 0x30) >> 2;// 55 to aa
    out[2] |= (buf[0] & 0x30) >> 4;// 11 to bb

    out[3] = buf[3] & 0xc0;// cc to cc
    out[3] |= (buf[2] & 0xc0) >> 2;// 88 to dd
    out[3] |= (buf[1] & 0xc0) >> 4;// 44 to ee
    out[3] |= (buf[0] & 0xc0) >> 6;// 00 to ff

    fprintf(stderr, " out %02x %02x %02x %02x ",
        out[0],
        out[1],
        out[2],
        out[3]
    );

    buf[0] = out[0];
    buf[1] = out[1];
    buf[2] = out[2];
    buf[3] = out[3];
}

int FormIEEE802154g::encode_bit(uint8_t bit)	/* FEC */
{
    char ui1, ui0;

    M2 = M1;
    M1 = M0;
    M0 = bi;
    bi = bit ? 1 : 0;

    ui1 = (bi + M1 + M2) % 2;
    ui0 = (bi + M0 + M1 + M2) % 2;

    if (ui1) {
        rf_buf[rf_buf_len] &= ~rf_bp;
        fprintf(stderr, "0");
    } else {
        rf_buf[rf_buf_len] |= rf_bp;
        fprintf(stderr, "1");
    }

    rf_bp >>= 1;
    if (rf_bp == 0) {
        rf_bp = 0x80;
        if (++rf_buf_len >= (int)sizeof(rf_buf))
            return -1;
        fprintf(stderr, " ");
    }

    if (ui0) {
        rf_buf[rf_buf_len] &= ~rf_bp;
        fprintf(stderr, "0");
    } else {
        rf_buf[rf_buf_len] |= rf_bp;
        fprintf(stderr, "1");
    }

    rf_bp >>= 1;
    if (rf_bp == 0) {
        rf_bp = 0x80;
        if (++rf_buf_len >= (int)sizeof(rf_buf))
            return -1;
        fprintf(stderr, " ");
    }

    return 0;
}

void FormIEEE802154g::shift_weights()
{
    weights[0] = weights[1];
    weights[1] = weights[2];
    weights[2] = weights[3];
    weights[3] = 0;
}

void FormIEEE802154g::print_weights()
{
    int i;
    for (i = NUM_WEIGHTS-1; i >= 0 ;i--) {
        fprintf(stderr, "%2d ", weights[i]);
    }
    fprintf(stderr, " : ");
}

void FormIEEE802154g::decode_ui(uint8_t ui)
{
    int w_bpo_p_1, w_bpo, w_bpo_m_1, w_bpo_m_2;

    ui &= 0x03;

    if (ui & 2) {
        if ( (pui >> 1) ^ (pui & 1) )
            w_bpo_p_1 = !(ppui >> 1) ^ (ppui & 1);
        else
            w_bpo_p_1 = (ppui >> 1) ^ (ppui & 1);
    } else {
        if ( (pui >> 1) ^ (pui & 1) )
            w_bpo_p_1 = (ppui >> 1) ^ (ppui & 1);
        else
            w_bpo_p_1 = !(ppui >> 1) ^ (ppui & 1);
    }

    w_bpo = (ui >> 1) ^ (ui & 1);
    w_bpo_m_1 = (pui >> 1) ^ (pui & 1);
    w_bpo_m_2 = (ppui >> 1) ^ (ppui & 1);
    fprintf(stderr, " {%d: %d %d %d %d} ", ui,
        w_bpo_p_1,
        w_bpo,
        w_bpo_m_1,
        w_bpo_m_2
    );

    if (w_bpo_p_1)
        weights[3]++;
    else
        weights[3]--;

    if (w_bpo)
        weights[2]++;
    else
        weights[2]--;

    if (w_bpo_m_1)
        weights[1]++;
    else
        weights[1]--;

    if (w_bpo_m_2)
        weights[0]++;
    else
        weights[0]--;

    ppui = pui;
    pui = ui;
}


void FormIEEE802154g::push_bit()
{
    if (weights[0] > 0) {
        phr_psdu_buf[phr_psdu_buf_idx] |= db_bp;
        fprintf(stderr, "1 @%02x %d", db_bp, phr_psdu_buf_idx);
    } else {
        phr_psdu_buf[phr_psdu_buf_idx] &= ~db_bp;
        fprintf(stderr, "0 @%02x %d", db_bp, phr_psdu_buf_idx);
    }

    db_bp >>= 1;
    if (db_bp == 0x00) {
        db_bp = 0x80;
        if (++phr_psdu_buf_idx >= (int)sizeof(phr_psdu_buf)) {
            fprintf(stderr, " push_bit phr_psdu_buf_idx\n");
            exit(-1);
        }
    }

}

void FormIEEE802154g::decode_byte(uint8_t o)
{
    decode_ui(o >> 6);
    print_weights();
    push_bit();
    shift_weights();
    fprintf(stderr, "\n");

    decode_ui(o >> 4);
    print_weights();
    push_bit();
    shift_weights();
    fprintf(stderr, "\n");

    decode_ui(o >> 2);
    print_weights();
    push_bit();
    shift_weights();
    fprintf(stderr, "\n");

    decode_ui(o);
    print_weights();
    push_bit();
    shift_weights();
    fprintf(stderr, "\n");

}

int FormIEEE802154g::do_fec_decode()
{
    int i, stop;

    db_bp = 0x80;
    phr_psdu_buf_idx = 0;

    for (i = NUM_WEIGHTS-1; i >= 0 ;i--)
        weights[i] = 0;

    interleave(&rf_buf[0]);

    /* this rf_buf is msbit first */
    ppui = rf_buf[0] >> 6;	// ui1
    ppui |= (rf_buf[0] & 0x40) >> 6;	// ui0

    pui = (rf_buf[0] & 0x20) >> 4;	// ui1
    pui |= (rf_buf[0] & 0x10) >> 4;	// ui0

    fprintf(stderr, "rf_buf[0]:%02x %d %d\n", rf_buf[0], ppui, pui);

    decode_ui(rf_buf[0] >> 2);
    print_weights();
    shift_weights();
    fprintf(stderr, "\n");

    decode_ui(rf_buf[0]);
    print_weights();
    push_bit();
    shift_weights();
    fprintf(stderr, "\n");

    rf_buf_len = 1;
    stop = 2;
    while (phr_psdu_buf_idx < stop) {
        fprintf(stderr, "rf_buf[%d] ", rf_buf_len);
        decode_byte(rf_buf[rf_buf_len++]);
        if (rf_buf_len >= 64) {
            fprintf(stderr, "decoding past fifo size\n");
            return -1;
        }
        if ((rf_buf_len & 0x03) == 0)
            interleave(&rf_buf[rf_buf_len]);
        if (stop == 2 && phr_psdu_buf_idx == 2) {
            phr.word = (phr_psdu_buf[0] << 8) | phr_psdu_buf[1];
            stop = phr.bits.frame_length + 2;
            fprintf(stderr, "\nphr.word:%04x stop:%d\n", phr.word, stop);
        }
    }

    fprintf(stderr, "\n(%d)phr_psdu_buf:", phr_psdu_buf_idx);
    for (i = 0; i < phr_psdu_buf_idx; i++)
        fprintf(stderr, "%02x ", phr_psdu_buf[i]);

    fprintf(stderr, "\n");

    return 0;
}

int FormIEEE802154g::do_fec_encode()
{
    int n;

    /* FEC is applied to PHR and PSDU as single block of data */

    M2 = 0;
    M1 = 0;
    M0 = 0;
    bi = 0;

    rf_buf_len = 0;
    rf_bp = 0x80;

    fprintf(stderr, "fec start phr_psdu_buf_idx:%d, ", phr_psdu_buf_idx);
    fprintf(stderr, " rf_buf_len=%d, bp=%02x\n", rf_buf_len, rf_bp);
    rf_bp = 0x80;
    for (n = 0; n < phr_psdu_buf_idx; n++) {
        uint8_t bp;
        fprintf(stderr, "{%02x}", phr_psdu_buf[n]);
        for (bp = 0x80; bp != 0; bp >>=1) {
            if (encode_bit(phr_psdu_buf[n] & bp)) {
                fprintf(stderr, "encode_bit fail\n");
                return -1;
            }
        }
        fprintf(stderr, " after byte: rf_buf_len=%d, bp=%02x ", rf_buf_len, rf_bp);
        if ((rf_buf_len & 0x03) == 0) {
            fprintf(stderr, "interleave %d", rf_buf_len-4);
            interleave(&rf_buf[rf_buf_len-4]);
        }
        fprintf(stderr, "\n");
    }

    fprintf(stderr, "t");
    if (encode_bit(0)) {	// tail
        fprintf(stderr, "encode_bit fail\n");
        return -1;
    }
    if (encode_bit(0)) {	// tail
        fprintf(stderr, "encode_bit fail\n");
        return -1;
    }
    if (encode_bit(0)) {	// tail
        fprintf(stderr, "encode_bit fail\n");
        return -1;
    }
    fprintf(stderr, " after tail: rf_buf_len=%d, bp=%02x\n", rf_buf_len, rf_bp);
    if (phr_psdu_buf_idx & 1) {
        fprintf(stderr, " ODD ");
        if (encode_bit(0)) {	// 5bit pad : 0
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 5bit pad : 1
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 5bit pad : 2
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 5bit pad : 3
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 5bit pad : 4
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
    } else {
        fprintf(stderr, " EVEN ");
        if (encode_bit(0)) {	// 13bit pad : 0
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 1
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 2
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 3
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 4
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 5
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 6
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 7
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 8
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 9
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(0)) {	// 13bit pad : 10
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 11
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
        if (encode_bit(1)) {	// 13bit pad : 12
            fprintf(stderr, "encode_bit fail\n	");
            return -1;
        }
    }

    fprintf(stderr, " after pad: rf_buf_len=%d, bp=%02x\n", rf_buf_len, rf_bp);
    if ((rf_buf_len & 0x03) != 0) {
        fprintf(stderr, "rf_buf_len not on 32bit boundary\n");
        return -1;
    }
    interleave(&rf_buf[rf_buf_len-4]);

    /* diag print */
    fprintf(stderr, "rf_buf: ");
    for (n = 0; n < rf_buf_len; n++)
        fprintf(stderr, "%02x ", rf_buf[n]);
    fprintf(stderr, "\n");
    return 0;
}

/* start_tx(): expects psdu_buf to contain payload to transmit,
 *      and psdu_buf_idx indicates its length without CRC */
void FormIEEE802154g::start_tx()
{
    RegIrqFlags2_t RegIrqFlags2;
    int n;
    uint8_t *tx_buf;	// pointer to actual buffer to be transmitted
    uint8_t tx_buf_len;	//  length of tx_buf

    RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
    if (!RegIrqFlags2.bits.FifoEmpty) {
        fprintf(stderr, "!FifoEmpty\n");
        return;
    }

    if (psdu_buf_idx_ < 0) {
        fprintf(stderr, "start_tx fail psdu_buf_idx_:%d\n", psdu_buf_idx_);
        return;
    }

    if (ui->checkBoxDWenable->isChecked()) {
        phr.bits.DW = 1;
        lfsr = 0x1ff;	// pn9 seed
        /* whitening is on PSDU only, not SHR or PHR */
    } else {
        phr.bits.DW = 0;
    }

    if (ui->checkBox2ByteFCS->isChecked()) {
        phr.bits.FCS = 1;
        MFR_length = 2;
    } else {
        phr.bits.FCS = 0;
        MFR_length = 4;
    }

    phr.bits.frame_length = psdu_buf_idx_ + MFR_length;


    phr_psdu_buf_idx = 0;
    phr_psdu_buf[phr_psdu_buf_idx++] = phr.word >> 8;
    phr_psdu_buf[phr_psdu_buf_idx++] = phr.word & 0xff;

    /* psdu data is already there, update the index */
    phr_psdu_buf_idx += psdu_buf_idx_;

    /* CRC is not done over PHR , only PSDU */
    if (phr.bits.FCS) {
        //using crc_msb_fist() because radio packet engine is msbit first
        gCRC = 	INITIAL_CRC16;
        gCRC = crc_msb_first(gCRC, psdu_buf_, psdu_buf_idx_);
        print_as_binary(gCRC, 16);
        phr_psdu_buf[phr_psdu_buf_idx++] = gCRC >> 8;
        phr_psdu_buf[phr_psdu_buf_idx++] = gCRC & 0xff;
    } else {
        uint8_t z;
        int i;
        crc_32 = INITIAL_CRC32;
        for (i = 0; i < psdu_buf_idx_; i++) {
            crc_32 = digital_update_crc32(crc_32, psdu_buf_+i, 1);
        }
        z = 0;
        while (i < 4) {
            fprintf(stderr, "Z");
            crc_32 = digital_update_crc32(crc_32, &z, 1);
            i++;
        }
        crc_32 = ~ crc_32;
        print_as_binary(crc_32, 32);
        phr_psdu_buf[phr_psdu_buf_idx++] = crc_32 >> 24;
        phr_psdu_buf[phr_psdu_buf_idx++] = crc_32 >> 16;
        phr_psdu_buf[phr_psdu_buf_idx++] = crc_32 >> 8;
        phr_psdu_buf[phr_psdu_buf_idx++] = crc_32 & 0xff;
    }

    if (phr.bits.DW) {
        for (n = PHR_LENGTH; n < phr_psdu_buf_idx; n++)	// start at 2: dont whiten PHR
            phr_psdu_buf[n] ^= get_pn9_byte();
    }

    if (ui->checkBoxFECenable->isChecked()) {
        fprintf(stderr, "tx FEC phr.word:%04x ", phr.word);
        if (do_fec_encode()) {
            fprintf(stderr, "do_fec_encode() fail\n");
            return;
        }
        SX127xFSK->RegPktConfig2.bits.PayloadLength = rf_buf_len;
        radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

        tx_buf = rf_buf;
        tx_buf_len = rf_buf_len;
    } else {
        SX127xFSK->RegPktConfig2.bits.PayloadLength = phr.bits.frame_length + PHR_LENGTH;
        radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

        tx_buf = phr_psdu_buf;
        tx_buf_len = phr_psdu_buf_idx;
    }

    // tx_buf is a pointer to actual buffer to be transmitted
    radio_fsk_write_fifo(tx_buf, tx_buf_len, false);

    set_opmode(RF_OPMODE_TRANSMITTER);
}

void FormIEEE802154g::create_tx_psdu()
{
    if (tx_crc_test) {
        psdu_buf_idx_ = 0;
        psdu_buf_[psdu_buf_idx_++] = 0x40;
        psdu_buf_[psdu_buf_idx_++] = 0x00;
        psdu_buf_[psdu_buf_idx_++] = 0x56;
    } else {
        int n, stop = ui->spinBoxPSDUlength->value() - MFR_length;

        if (stop < 0) {
            fprintf(stderr, "length less than MFR_length\n");
            return;
        }

        psdu_buf_idx_ = 0;
        for (n = 0; n < stop; n++) {
            if (payload_octet == -1) {
                psdu_buf_[psdu_buf_idx_++] = tx_cnt;
            } else
                psdu_buf_[psdu_buf_idx_++] = payload_octet;
        }
        if (payload_octet == -1)
            tx_cnt++;
    }
}

void FormIEEE802154g::on_PushButtonStartTX_clicked()
{
    if (tx_tid != 0)
        return;

    tx_crc_test = false;
    create_tx_psdu();

    if (ui->checkBoxTXrepeatEnable->isChecked()) {
        num_txs = ui->spinBoxTXcount->value();
        if (num_txs > 0)
            tx_tid = startTimer(ui->spinBoxTXinterval->value());
    } else
        start_tx();
}

void FormIEEE802154g::parse_rx_pkt()
{
    int i;
    uint8_t *rx_buf_ptr;

    if (ui->checkBoxFECenable->isChecked()) {
        /* rf_buf contains encoded packet */
        if (do_fec_decode()) {
            fprintf(stderr, "do_fec_decode() failed\n");
            return;
        }
        /* phr_psdu_buf contains decoded packet */
        rx_buf_ptr = phr_psdu_buf;
    } else {
        /* rf_buf contains the packet */
        phr.word = rf_buf[0] << 8;
        phr.word += rf_buf[1];
        fprintf(stderr, "frame_length:%d : ", phr.bits.frame_length);
        for (i = 0; i < (phr.bits.frame_length+2); i++)
            fprintf(stderr, "%02x ", rf_buf[i]);
        fprintf(stderr, "\n");
        rx_buf_ptr = rf_buf;
    }

    if (phr.bits.DW) {
        lfsr = 0x1ff;	// pn9 seed
        fprintf(stderr, "rx-whitening ");
        for (i = 2; i <(phr.bits.frame_length+2); i++)
            rx_buf_ptr[i] ^= get_pn9_byte();
    }

    if (phr.bits.FCS) {
        fprintf(stderr, "16bit-FCS  ");
        gCRC = 	INITIAL_CRC16;
        gCRC = crc_msb_first(gCRC, rx_buf_ptr+2, phr.bits.frame_length);
        if (gCRC == 0)
            fprintf(stderr, "CRC16-ok good=%d ", ++crc_good_count);
        else {
            fprintf(stderr, " FAIL ");
            print_as_binary(gCRC, 16);
        }
    } else {
        uint8_t z;
        uint32_t rx_crc;
        fprintf(stderr, "32bit-FCS  ");
        crc_32 = INITIAL_CRC32;
        for (i = 0; i < phr.bits.frame_length-4; i ++)
            crc_32 = digital_update_crc32(crc_32, rx_buf_ptr+i+2, 1);
        z = 0;
        while (i < 4) {
            crc_32 = digital_update_crc32(crc_32, &z, 1);
            i++;
        }
        crc_32 = ~crc_32;
        print_as_binary(crc_32, 32);
        // +2 because firs two bytes of rx_buf_ptr is PHR
        rx_crc = rx_buf_ptr[phr.bits.frame_length-2] << 24;
        rx_crc |= rx_buf_ptr[phr.bits.frame_length-1] << 16;
        rx_crc |= rx_buf_ptr[phr.bits.frame_length] << 8;
        rx_crc |= rx_buf_ptr[phr.bits.frame_length+1];
        if (crc_32 == rx_crc)
            fprintf(stderr, "ok crc_32:%08x, rx_crc:%08x good:%d\n", crc_32, rx_crc, crc_good_count++);
        else
            fprintf(stderr, "FAIL crc_32:%08x, rx_crc:%08x\n", crc_32, rx_crc);
    }

    fprintf(stderr, "\n");
}

static int cnt;

void FormIEEE802154g::timerEvent(QTimerEvent* event)
{
    int tid = event->timerId();
    if (tid == pn9test_tid) {
        while (!digitalRead(DIO2_PIN))	// while fifo not full
            radio_write(REG_FIFO, get_pn9_byte());
    } else if (tid == rx_tid) {
        if (flags.new_rx_pkt) {
            int i;
            flags.new_rx_pkt = 0;
            for (i = 0; i < rf_buf_len; i++) {
                fprintf(stderr, "%02x ", rf_buf[i]);
            }
            fprintf(stderr, "\n");
            parse_rx_pkt();
        }
    } else if (tid == tx_tid) {
        fprintf(stderr, "num_txs%d\n", num_txs);
        create_tx_psdu();
        start_tx();
        if (--num_txs == 0) {
            killTimer(tx_tid);
            tx_tid = 0;
        }
    }
}

void FormIEEE802154g::on_pushButtonTXPN9_clicked(bool checked)
{
    RegIrqFlags2_t RegIrqFlags2;

    if (checked) {
        set_opmode(RF_OPMODE_STANDBY);
        RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
        while (!RegIrqFlags2.bits.FifoEmpty) {
            (void)radio_read(REG_FIFO);
            RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
        }
        if (SX127x.RegDioMapping1.bits.Dio2Mapping == 1) {	// fifofull on DIO2
            SX127x.RegDioMapping1.bits.Dio2Mapping = 0;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
        }
        if (SX127x.RegDioMapping1.bits.Dio0Mapping != 2) {	// DIO0 shutup
            SX127x.RegDioMapping1.bits.Dio0Mapping = 2;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
        }
        // start unlimited length transmit
        SX127xFSK->RegPktConfig2.bits.PayloadLength = 0;
        radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);
        lfsr = 0x1ff;	// pn9 seed

        cnt = 0;
        while (!digitalRead(DIO2_PIN)) { 	// while fifo not full
            uint8_t o = get_pn9_byte();
            radio_write(REG_FIFO, o);
            if (cnt++ > 70) {
                fprintf(stderr, "dio2 fail\n");
                return;
            }
        }
        set_opmode(RF_OPMODE_TRANSMITTER);
        pn9test_tid = startTimer(10);
    } else {
        // stop transmit
        fprintf(stderr, "pn9stop\n");
        set_opmode(RF_OPMODE_STANDBY);
        flags.pn9_tx_test_ = 0;
        killTimer(pn9test_tid);
    }
}

void FormIEEE802154g::set_sfd(bool fec)
{
    char str[16];
    uint16_t sfd;

    SX127xFSK->RegSyncValues[0] = 0x55;
    if (fec) {
        sfd = SFD_FEC;
    } else {
        sfd = SFD_UNCODED;
    }

    if (ui->checkBoxWrongBitEnable->isChecked()) {
        uint16_t bp = 1 << ui->spinBoxWrongBit->value();
        fprintf(stderr, "bp:%04x\n", bp);
        if (sfd & bp)
            sfd &= ~bp;
        else
            sfd |= bp;
    }

    SX127xFSK->RegSyncValues[1] = sfd >> 8;
    SX127xFSK->RegSyncValues[2] = sfd & 0xff;

    radio_write(REG_FSK_SYNCVALUE1, SX127xFSK->RegSyncValues[0]);
    radio_write(REG_FSK_SYNCVALUE2, SX127xFSK->RegSyncValues[1]);
    radio_write(REG_FSK_SYNCVALUE3, SX127xFSK->RegSyncValues[2]);

    sprintf(str, "0x%04x", sfd);
    ui->labelSFD->setText(str);
}

void FormIEEE802154g::on_checkBoxFECenable_toggled(bool checked)
{
    set_sfd(checked);
}

void FormIEEE802154g::print_as_binary(uint32_t v, int num_bits)
{
    char str[64], *strp;
    uint32_t bp;

    strp = str;
    for (bp = 1 << (num_bits-1); bp != 0; bp >>= 1) {
        if (v & bp)
            *strp++ = '1';
        else
            *strp++ = '0';
    }
    *strp++ = ' ';
    *strp = 0;

    fprintf(stderr, str);
}

uint16_t FormIEEE802154g::crc_msb_first(uint16_t crc, uint8_t const *p, int len)
{
    while (len--) {
        unsigned int i;
        print_as_binary(*p, 8);
        crc ^= (uint16_t)(uint8_t)*p++ << 8;
        for (i = 0; i < 8; i++)
            crc = (crc << 1) ^ ((crc & 0x8000) ? POLY_ITUT : 0);
    }
    return crc;
}
// Automatically generated CRC function
// polynomial: 0x104C11DB7
unsigned int FormIEEE802154g::digital_update_crc32(unsigned int crc, const unsigned char *data, size_t len)
{
    static const unsigned int table[256] = {
    0x00000000U,0x04C11DB7U,0x09823B6EU,0x0D4326D9U,
    0x130476DCU,0x17C56B6BU,0x1A864DB2U,0x1E475005U,
    0x2608EDB8U,0x22C9F00FU,0x2F8AD6D6U,0x2B4BCB61U,
    0x350C9B64U,0x31CD86D3U,0x3C8EA00AU,0x384FBDBDU,
    0x4C11DB70U,0x48D0C6C7U,0x4593E01EU,0x4152FDA9U,
    0x5F15ADACU,0x5BD4B01BU,0x569796C2U,0x52568B75U,
    0x6A1936C8U,0x6ED82B7FU,0x639B0DA6U,0x675A1011U,
    0x791D4014U,0x7DDC5DA3U,0x709F7B7AU,0x745E66CDU,
    0x9823B6E0U,0x9CE2AB57U,0x91A18D8EU,0x95609039U,
    0x8B27C03CU,0x8FE6DD8BU,0x82A5FB52U,0x8664E6E5U,
    0xBE2B5B58U,0xBAEA46EFU,0xB7A96036U,0xB3687D81U,
    0xAD2F2D84U,0xA9EE3033U,0xA4AD16EAU,0xA06C0B5DU,
    0xD4326D90U,0xD0F37027U,0xDDB056FEU,0xD9714B49U,
    0xC7361B4CU,0xC3F706FBU,0xCEB42022U,0xCA753D95U,
    0xF23A8028U,0xF6FB9D9FU,0xFBB8BB46U,0xFF79A6F1U,
    0xE13EF6F4U,0xE5FFEB43U,0xE8BCCD9AU,0xEC7DD02DU,
    0x34867077U,0x30476DC0U,0x3D044B19U,0x39C556AEU,
    0x278206ABU,0x23431B1CU,0x2E003DC5U,0x2AC12072U,
    0x128E9DCFU,0x164F8078U,0x1B0CA6A1U,0x1FCDBB16U,
    0x018AEB13U,0x054BF6A4U,0x0808D07DU,0x0CC9CDCAU,
    0x7897AB07U,0x7C56B6B0U,0x71159069U,0x75D48DDEU,
    0x6B93DDDBU,0x6F52C06CU,0x6211E6B5U,0x66D0FB02U,
    0x5E9F46BFU,0x5A5E5B08U,0x571D7DD1U,0x53DC6066U,
    0x4D9B3063U,0x495A2DD4U,0x44190B0DU,0x40D816BAU,
    0xACA5C697U,0xA864DB20U,0xA527FDF9U,0xA1E6E04EU,
    0xBFA1B04BU,0xBB60ADFCU,0xB6238B25U,0xB2E29692U,
    0x8AAD2B2FU,0x8E6C3698U,0x832F1041U,0x87EE0DF6U,
    0x99A95DF3U,0x9D684044U,0x902B669DU,0x94EA7B2AU,
    0xE0B41DE7U,0xE4750050U,0xE9362689U,0xEDF73B3EU,
    0xF3B06B3BU,0xF771768CU,0xFA325055U,0xFEF34DE2U,
    0xC6BCF05FU,0xC27DEDE8U,0xCF3ECB31U,0xCBFFD686U,
    0xD5B88683U,0xD1799B34U,0xDC3ABDEDU,0xD8FBA05AU,
    0x690CE0EEU,0x6DCDFD59U,0x608EDB80U,0x644FC637U,
    0x7A089632U,0x7EC98B85U,0x738AAD5CU,0x774BB0EBU,
    0x4F040D56U,0x4BC510E1U,0x46863638U,0x42472B8FU,
    0x5C007B8AU,0x58C1663DU,0x558240E4U,0x51435D53U,
    0x251D3B9EU,0x21DC2629U,0x2C9F00F0U,0x285E1D47U,
    0x36194D42U,0x32D850F5U,0x3F9B762CU,0x3B5A6B9BU,
    0x0315D626U,0x07D4CB91U,0x0A97ED48U,0x0E56F0FFU,
    0x1011A0FAU,0x14D0BD4DU,0x19939B94U,0x1D528623U,
    0xF12F560EU,0xF5EE4BB9U,0xF8AD6D60U,0xFC6C70D7U,
    0xE22B20D2U,0xE6EA3D65U,0xEBA91BBCU,0xEF68060BU,
    0xD727BBB6U,0xD3E6A601U,0xDEA580D8U,0xDA649D6FU,
    0xC423CD6AU,0xC0E2D0DDU,0xCDA1F604U,0xC960EBB3U,
    0xBD3E8D7EU,0xB9FF90C9U,0xB4BCB610U,0xB07DABA7U,
    0xAE3AFBA2U,0xAAFBE615U,0xA7B8C0CCU,0xA379DD7BU,
    0x9B3660C6U,0x9FF77D71U,0x92B45BA8U,0x9675461FU,
    0x8832161AU,0x8CF30BADU,0x81B02D74U,0x857130C3U,
    0x5D8A9099U,0x594B8D2EU,0x5408ABF7U,0x50C9B640U,
    0x4E8EE645U,0x4A4FFBF2U,0x470CDD2BU,0x43CDC09CU,
    0x7B827D21U,0x7F436096U,0x7200464FU,0x76C15BF8U,
    0x68860BFDU,0x6C47164AU,0x61043093U,0x65C52D24U,
    0x119B4BE9U,0x155A565EU,0x18197087U,0x1CD86D30U,
    0x029F3D35U,0x065E2082U,0x0B1D065BU,0x0FDC1BECU,
    0x3793A651U,0x3352BBE6U,0x3E119D3FU,0x3AD08088U,
    0x2497D08DU,0x2056CD3AU,0x2D15EBE3U,0x29D4F654U,
    0xC5A92679U,0xC1683BCEU,0xCC2B1D17U,0xC8EA00A0U,
    0xD6AD50A5U,0xD26C4D12U,0xDF2F6BCBU,0xDBEE767CU,
    0xE3A1CBC1U,0xE760D676U,0xEA23F0AFU,0xEEE2ED18U,
    0xF0A5BD1DU,0xF464A0AAU,0xF9278673U,0xFDE69BC4U,
    0x89B8FD09U,0x8D79E0BEU,0x803AC667U,0x84FBDBD0U,
    0x9ABC8BD5U,0x9E7D9662U,0x933EB0BBU,0x97FFAD0CU,
    0xAFB010B1U,0xAB710D06U,0xA6322BDFU,0xA2F33668U,
    0xBCB4666DU,0xB8757BDAU,0xB5365D03U,0xB1F740B4U,
    };

    while (len > 0)
    {
        print_as_binary(*data, 8);
      crc = table[*data ^ ((crc >> 24) & 0xff)] ^ (crc << 8);
        fprintf(stderr, " (%02x:%08x) ", *data, crc);
      data++;
      len--;
    }
    return crc;
}

void FormIEEE802154g::on_pushButtonTXcrcTest_clicked()
{
    if (tx_tid != 0)
        return;

    tx_crc_test = true;
    create_tx_psdu();

    if (ui->checkBoxTXrepeatEnable->isChecked()) {
        num_txs = ui->spinBoxTXcount->value();
        if (num_txs > 0)
            tx_tid = startTimer(ui->spinBoxTXinterval->value());
    } else
        start_tx();
}

void FormIEEE802154g::on_checkBox2ByteFCS_clicked(bool checked)
{
    if (checked) {
        // 16bit crc
        ui->spinBoxPSDUlength->setMinimum(2);
        if (ui->spinBoxPSDUlength->value() < 2)
            ui->spinBoxPSDUlength->setValue(2);
        MFR_length = 2;
    } else {
        // 32bit crc
        ui->spinBoxPSDUlength->setMinimum(4);
        if (ui->spinBoxPSDUlength->value() < 4)
            ui->spinBoxPSDUlength->setValue(4);
        MFR_length = 4;
    }
}

void FormIEEE802154g::on_lineEditPayloadOctet_textChanged(const QString &arg1)
{
    QByteArray barray = arg1.toLocal8Bit();
    char *data = barray.data();
    fprintf(stderr, "len:%d\n", strlen(data));
    if (strlen(data) == 0)
        payload_octet = -1;
    else {
        payload_octet = strtol(data, NULL, 0);
        fprintf(stderr, "payload_octet:%02x\n", payload_octet);
    }

}

/*void FormIEEE802154g::test_interleave()
{
    uint8_t buf[4];

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[0] = 0xc0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[0] = 0x30;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[0] = 0x0c;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[0] = 0x03;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[1] = 0xc0;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[1] = 0x30;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[1] = 0x0c;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[1] = 0x03;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[2] = 0xc0;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[2] = 0x30;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[2] = 0x0c;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[2] = 0x03;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[3] = 0xc0;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[3] = 0x30;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[3] = 0x0c;
    interleave(buf);

    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    buf[3] = 0x03;
    interleave(buf);
}*/

void FormIEEE802154g::on_pushButtonStartRX_clicked(bool checked)
{
    if (checked) {
        SX127xFSK->RegPktConfig2.bits.PayloadLength = 64;
        radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

        set_opmode(RF_OPMODE_RECEIVER);
        rx_tid = startTimer(20);
        crc_good_count = 0;
    } else {
        set_opmode(RF_OPMODE_STANDBY);
        killTimer(rx_tid);
    }

}

void FormIEEE802154g::on_comboBoxChannel_currentIndexChanged(int index)
{
    float MHz = ui->comboBoxChannel->itemData(index).toFloat();
    fprintf(stderr, "%d: %f\n", index, MHz);
    setMHz(MHz);
}

void FormIEEE802154g::on_checkBoxWrongBitEnable_clicked(bool checked)
{
    ui->spinBoxWrongBit->setEnabled(checked);
    ui->labelWrongSFD->setEnabled(checked);
    set_sfd(ui->checkBoxFECenable->isChecked());
}

void FormIEEE802154g::on_spinBoxWrongBit_valueChanged(int arg1)
{
	(void)arg1;
    set_sfd(ui->checkBoxFECenable->isChecked());
}

void FormIEEE802154g::on_pushButtonFECones_clicked()
{
    rf_buf_len = 0;
    rf_buf[rf_buf_len++] = 0x00;
    rf_buf[rf_buf_len++] = 0x00;
    rf_buf[rf_buf_len++] = 0x00;

    SX127xFSK->RegPktConfig2.bits.PayloadLength = rf_buf_len;
    radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

    radio_fsk_write_fifo(rf_buf, rf_buf_len, false);
    set_opmode(RF_OPMODE_TRANSMITTER);
}

void FormIEEE802154g::on_checkBoxTXrepeatEnable_clicked(bool checked)
{
    ui->labelTXinterval->setEnabled(checked);
    ui->labelTXcount->setEnabled(checked);
    ui->spinBoxTXinterval->setEnabled(checked);
    ui->spinBoxTXcount->setEnabled(checked);

    if (!checked && tx_tid != 0) {
        killTimer(tx_tid);
        tx_tid = 0;
    }
}

