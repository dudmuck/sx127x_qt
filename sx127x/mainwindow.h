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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdio.h>
#include "spi.h"
#include "cfg.h"

#include <QMainWindow>
#include <QModelIndex>

#include <QLabel>

#define STOP_MONITOR    0x01
#define STOP_TX_RUN     0x02
#define STOP_RX_RUN     0x04
#define STOP_ALL        (STOP_MONITOR | STOP_TX_RUN | STOP_RX_RUN)

#define DIO_POLL_RATE		50

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void timerEvent(QTimerEvent *);
    
private slots:
    void on_actionLoRa_mode_triggered();

    void on_actionFSK_triggered();

    void on_actionCommon_triggered();

    void on_radioButtonSleep_clicked();

    void on_radioButtonStandby_clicked();

    void on_radioButtonFSRX_clicked();

    void on_radioButtonFSTX_clicked();

    void on_radioButtonRX_clicked();

    void on_radioButtonTX_clicked();

    void on_doubleSpinBoxFrf_valueChanged(double arg1);

    void on_comboBoxDataMode_currentIndexChanged(int index);

    void on_comboBoxTXpin_currentIndexChanged(int index);

    void on_spinBoxdBmout_valueChanged(int arg1);

    void on_checkBoxPADAC_clicked(bool checked);

    void on_actionHW_reset_triggered();

    void on_actionRead_all_triggered();

    void on_listWidgetDIO5FSK_clicked(const QModelIndex &index);

    void on_listWidgetDIO4FSK_clicked(const QModelIndex &index);

    void on_listWidgetDIO3FSK_clicked(const QModelIndex &index);

    void on_listWidgetDIO2FSK_clicked(const QModelIndex &index);

    void on_listWidgetDIO1FSK_clicked(const QModelIndex &index);

    void on_listWidgetDIO0FSK_clicked(const QModelIndex &index);

    void on_actionMonitor_triggered(bool checked);

    void on_spinBoxBitrate_editingFinished();

    void on_spinBoxFdev_editingFinished();

    void on_comboBoxOOK_activated(int index);

    void on_comboBoxShaping_activated(int index);

    void on_checkBoxAfcAutoOn_clicked(bool checked);

    void on_checkBoxAfcAutoClearOn_clicked(bool checked);

    void on_pushButtonAfcClear_clicked();

    void on_checkBoxPreambleDetEnable_clicked(bool checked);

    void on_comboBoxPreambleDetSize_currentIndexChanged(int index);

    void on_spinBoxPreambleErrTol_editingFinished();

    void on_comboBoxRxTrigger_currentIndexChanged(int index);

    void on_plainTextEdit_payload_ascii_textChanged();

    void on_plainTextEdit_payload_hex_textChanged();

    void on_pushButtonPktTXRun_clicked(bool checked);

    void on_comboBoxFSKPktFormat_currentIndexChanged(int index);

    void on_comboBoxTxStartCond_currentIndexChanged(int index);

    void on_pushButtonPktRXRun_clicked(bool checked);

    void on_checkBoxFSKCrCOn_clicked(bool checked);

    void on_spinBoxFSKFifoThreshold_editingFinished();

    void on_spinBoxFSKSyncSize_valueChanged(int arg1);

    void on_lineEditFSKSyncWord_editingFinished();

    void on_spinBoxRegPayloadLength_valueChanged(int arg1);

    void on_actionOpen_Config_triggered();

    void on_actionSave_Config_triggered();

    void on_actionSave_Config_As_triggered();

    void on_checkBoxFSKSyncOn_clicked(bool checked);

    void on_comboBoxFSKPreamblePolarity_currentIndexChanged(int index);

    void on_checkBoxFSKCrcAutoClearOff_clicked(bool checked);

    void on_comboBoxFSKAutoRestartRxMode_currentIndexChanged(int index);

    void on_spinBoxFSKPreambleSize_editingFinished();

    void on_comboBoxFSKFIfoFillCondition_currentIndexChanged(int index);

    void on_comboBoxFSKCrcWhiteningType_currentIndexChanged(int index);

    void on_comboBoxFSKDcFree_currentIndexChanged(int index);

    void on_comboBoxFSKAdrsFilt_currentIndexChanged(int index);

    void on_lineEditFSKNodeAdrs_editingFinished();

    void on_lineEditFSKBcastAdrs_editingFinished();

    void on_lineEditFSKAddrToSend_editingFinished();

    void on_comboBoxMapPreambleDetect_currentIndexChanged(int index);

    void on_checkBoxRestartRxOnCollision_clicked(bool checked);

    void on_pushButtonRestartRxWithoutPllLock_clicked();

    void on_pushButtonRestartRxWithPllLock_clicked();

    void on_doubleSpinBoxRssiThreshold_editingFinished();

    void on_comboBoxRssiSmoothing_currentIndexChanged(int index);

    void on_spinBoxRssiOffset_valueChanged(int arg1);

    void on_pushButtonAgcStart_clicked();

    void on_checkBoxAgcAutoOn_clicked(bool checked);

    void on_spinBoxFracBR_editingFinished();

    void on_comboBoxTXpllBW_currentIndexChanged(int index);

    void on_radioButtonCAD_clicked();

    void on_radioButtonRxSingle_clicked();

    void on_comboBoxPAramp_currentIndexChanged(int index);

    void on_comboBoxLNAboostLF_currentIndexChanged(int index);

    void on_comboBoxLNAboostHF_currentIndexChanged(int index);

    void on_checkBoxOcpOn_clicked(bool checked);

    void on_spinBoxOCPtrim_valueChanged(int arg1);

    void on_checkBoxLowPnTxPllOff_clicked(bool checked);

    void on_comboBoxPLLbwHF_currentIndexChanged(int index);

    void on_comboBoxPLLbwLF_currentIndexChanged(int index);

    void on_comboBoxRXBW_currentIndexChanged(int index);

    void on_comboBoxAfcBW_currentIndexChanged(int index);

    void on_checkBoxLowBatOn_clicked(bool checked);

    void on_comboBoxLowBatTrim_currentIndexChanged(int index);

    void on_checkBoxAutoImageCalOn_clicked(bool checked);

    void on_pushButtonImageCalStart_clicked();

    void on_checkBoxTempMonitorOff_clicked(bool checked);

    void on_comboBoxTempThreshold_currentIndexChanged(int index);

    void on_tabWidgetFSK_currentChanged(int index);

    void on_stackedWidget_currentChanged(int arg1);

private:
    Ui::MainWindow *ui;
    Spi spi;
    Cfg cfg;
    void getRadioStatus(void);
    void getRadioStatus_fsk(void);
    int monitor_tid, rx_tid, diopoll_tid, startup_tid;
    int list_DataModePacket;
    void regen_fsk_dio_lists(void);
    void read_fsk_dio(void);
    void update_dBmout_spinbox(void);
   /*uint32_t get_rx_bw_hz(uint8_t addr);
    void set_rx_dcc_bw_hz(uint32_t bw_hz, char afc);
    void ComputeRxBwMantExp(uint32_t rxBwValue, uint8_t* mantisse, uint8_t* exponent);
    uint32_t ComputeRxBw( uint8_t mantisse, uint8_t exponent );*/
    uint32_t get_bitrate(void);
    uint32_t get_tx_fdev_hz(void);
    bool text_changed_busy;
    void get_irq_flags(void);
    void fsk_enable_packet_mode(bool en);
    int _fsk_start_tx(void);
    void start_tx(void);
    void stop_(uint8_t f);
    void fsk_set_dio0_rx(void);
    void read_fsk_sync_word(void);
    void set_SyncSize(int v);
    void fill_hex_TextEdit(uint8_t* buf, int buf_len);
    void setup_FifoLevel(edge_e edge);
    void AddressFilteringChanged(void);
    void update_labelFSKPreamble(void);
    void id_radio(void);
	//void showOpMode(void);
	void sx1272_show_pllbw(void);
	void init_rxbw(void);
	float bws[24];
	int num_bws;
	bool init_done;
	void monitor_temperature(void);
	void startup(void);

    QLabel *m_statusLabel;
};

#endif // MAINWINDOW_H
