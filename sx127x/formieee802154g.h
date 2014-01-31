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
#ifndef FORMIEEE802154G_H
#define FORMIEEE802154G_H
#include <stdint.h>
#include <QWidget>

#define NUM_WEIGHTS		4

namespace Ui {
class FormIEEE802154g;
}

class FormIEEE802154g : public QWidget
{
    Q_OBJECT
    
public:
    explicit FormIEEE802154g(QWidget *parent = 0);
    ~FormIEEE802154g();
    void timerEvent(QTimerEvent *);
    
private slots:
    void on_comboBoxInit_currentIndexChanged(int index);

    void on_PushButtonStartTX_clicked();

    void on_pushButtonTXPN9_clicked(bool checked);

    void on_checkBoxFECenable_toggled(bool checked);

    void on_pushButtonTXcrcTest_clicked();

    void on_checkBox2ByteFCS_clicked(bool checked);

    void on_lineEditPayloadOctet_textChanged(const QString &arg1);

    void on_pushButtonStartRX_clicked(bool checked);

    void on_comboBoxChannel_currentIndexChanged(int index);

    void on_checkBoxWrongBitEnable_clicked(bool checked);

    void on_spinBoxWrongBit_valueChanged(int arg1);

    void on_pushButtonFECones_clicked();

    void on_checkBoxTXrepeatEnable_clicked(bool checked);

private:
    Ui::FormIEEE802154g *ui;
    void start_tx(void);
    int pn9test_tid, rx_tid, tx_tid;
    uint8_t reverse_octet(uint8_t octet);
    void set_sfd(bool fec);
    unsigned int tx_cnt;
    uint16_t crc_msb_first(uint16_t crc, uint8_t const *p, int len);
    uint16_t gCRC;
    unsigned int digital_update_crc32(unsigned int crc, const unsigned char *data, size_t len);
    //unsigned int digital_crc32(const unsigned char *buf, unsigned int len);
    uint8_t phr_psdu_buf[64];
    uint8_t *psdu_buf_;
    int psdu_buf_idx_;	// this is the index of psdu_buf ptr
    int phr_psdu_buf_idx;	// this is the index of phr_psdu_buf[]
    void print_as_binary(uint32_t octet, int num_bits);
    int payload_octet;
    uint32_t crc_32;
    void parse_rx_pkt(void);
    uint8_t MFR_length;

    int do_fec_decode(void);	// FEC decoder
    uint8_t ppui, pui;	// FEC decoder
    void decode_ui(uint8_t);
    void decode_byte(uint8_t);	// FEC decoder
    int weights[NUM_WEIGHTS];	// FEC decoder
    void print_weights(void);	// FEC decoder (observation)
    void shift_weights(void);	// FEC decoder
    void push_bit(void);	// FEC decoder
    uint8_t db_bp;		// FEC decoder (bit position in decoded buf)

    int do_fec_encode(void);	// FEC
    int encode_bit(uint8_t);	// FEC
    char M2, M1, M0, bi;	// FEC
    /*uint8_t rf_buf[64];	// already in spi.h
    uint8_t rf_buf_idx;	// already in spi.h*/
    uint8_t rf_bp;
    void interleave(uint8_t*);
    void test_interleave(void);
    unsigned int crc_good_count;
    void create_tx_psdu(void);
    bool tx_crc_test;
    int num_txs;
};

#endif // FORMIEEE802154G_H
