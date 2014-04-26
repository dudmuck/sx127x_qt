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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <math.h>

//#include "formlora.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_statusLabel = new QLabel(this);
    ui->statusBar->addWidget(m_statusLabel);

	spi_radioButtonTX = ui->radioButtonTX;
	spi_radioButtonStandby = ui->radioButtonStandby;
	spi_radioButtonSleep = ui->radioButtonSleep;
	spi_radioButtonFSTX = ui->radioButtonFSTX;
	spi_radioButtonFSRX = ui->radioButtonFSRX;
	spi_radioButtonRX = ui->radioButtonRX;
	spi_radioButtonRxSingle = ui->radioButtonRxSingle;
	spi_radioButtonCAD = ui->radioButtonCAD;

    spi_checkBoxPayloadCrcError = ui->checkBoxPayloadCrcError;
    spi_checkBoxTxDone = ui->checkBoxTxDone;
    spi_checkBoxFHSS = ui->checkBoxFHSS;
    spi_checkBoxRxTimeout = ui->checkBoxRxTimeout;
	spi_checkBoxRxDone = ui->checkBoxRxDone;
    spi_checkBoxValidHeader = ui->checkBoxValidHeader;
    spi_checkBoxCadDone = ui->checkBoxCadDone;
    spi_checkBoxCadDetected = ui->checkBoxCadDetected;

	xtal_hz = &cfg.xtal_hz;

    text_changed_busy = false;
    rf_buf_len = 0;

    list_DataModePacket = -1;
	init_done = false;

    resize(minimumWidth(), minimumHeight());

	/* initialization done after wiringPi startup.. */
    startup_tid = startTimer(1000);
}

void MainWindow::startup()
{
    if (spi.open()) {
		fprintf(stderr, "spi.open() failed\n");
		m_statusLabel->setText(QApplication::translate("MainWindow", "spi.open() failed", 0, QApplication::UnicodeUTF8));
		return;
	}

    id_radio();
    ui->widget_lora->init();

	init_rxbw();
    getRadioStatus();

	if (ui->stackedWidget->currentIndex() == 1) {
		// starting on FSK page
		if (ui->tabWidgetFSK->currentIndex() == 2) {
			diopoll_tid = startTimer(DIO_POLL_RATE);
		}
	} else {
		// starting on LoRa page
	}
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::init_rxbw()
{
	int exp;

	ui->comboBoxRXBW->clear();
	ui->comboBoxAfcBW->clear();

	num_bws = 0;
	for (exp = 7; exp > 0; exp--) {
		int mant;
		for (mant = 24; mant > 12; mant -= 4) {
			char str[32];
			float rxbw = *xtal_hz / (mant * (1<<(exp+2)));
			sprintf(str, "%.2fKHz", rxbw/1000);
			ui->comboBoxRXBW->addItem(str);
			ui->comboBoxAfcBW->addItem(str);
			bws[num_bws++] = rxbw;
		}
	}

	init_done = true;
}

void MainWindow::id_radio()
{
    /* identify if SX1276 is plugged in or SX1272 */
    RegOpMode_t orig_regopmode, test_regopmode;
	char str[64];

    SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
    orig_regopmode.octet = SX127x.RegOpMode.octet;

	/* LowFrequencyModeOn doesnt exist in sx1272 when in lora mode */
    if (!orig_regopmode.fsk_bits.LongRangeMode) {
        if (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP) {
            SX127x.RegOpMode.fsk_bits.Mode = RF_OPMODE_SLEEP;
            radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
			do {
				SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
			} while (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP);
		}
        SX127x.RegOpMode.fsk_bits.LongRangeMode = 1;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

    SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
    SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276 ^= 1;
    radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
    test_regopmode.octet = radio_read(REG_OPMODE);
    if (test_regopmode.bits.LowFrequencyModeOn_sx1276 == SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276) {
        sx1276 = true;
		sprintf(str, "RegVersion:0x%02x chip type:SX1276", radio_read(REG_VERSION));
    } else {
        sx1276 = false;
		sprintf(str, "RegVersion:0x%02x chip type:SX1272", radio_read(REG_VERSION));
    }

    m_statusLabel->setText(QApplication::translate("MainWindow", str, 0, QApplication::UnicodeUTF8));

    if (!orig_regopmode.fsk_bits.LongRangeMode) {
        if (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP) {
            SX127x.RegOpMode.fsk_bits.Mode = RF_OPMODE_SLEEP;
            radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
			do {
				SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
			} while (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP);
			fprintf(stderr, "R-too sleep readback %02x\n", SX127x.RegOpMode.octet);
		}
		SX127x.RegOpMode.octet = orig_regopmode.octet;
        SX127x.RegOpMode.bits.Mode = RF_OPMODE_SLEEP;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

        SX127x.RegOpMode.bits.Mode = orig_regopmode.bits.Mode;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

		SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
	}

	/*****************************************/
	ui->labelLNAboostLF->setEnabled(sx1276);
	ui->comboBoxLNAboostLF->setEnabled(sx1276);
	ui->checkBoxLowPnTxPllOff->setEnabled(!sx1276);
	ui->labelPLLbwLF->setEnabled(sx1276);	// sx1276 in reg 0x70
	ui->comboBoxPLLbwLF->setEnabled(sx1276);
}

void MainWindow::on_actionLoRa_mode_triggered()
{
    ui->stackedWidget->setCurrentIndex(0);
    ui->stackedWidgetIrqFlags->setCurrentIndex(0);

    SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
    if (!SX127x.RegOpMode.fsk_bits.LongRangeMode) {
        if (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP) {
            SX127x.RegOpMode.fsk_bits.Mode = RF_OPMODE_SLEEP;
            radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
        }
        SX127x.RegOpMode.fsk_bits.LongRangeMode = 1;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

        stop_(STOP_ALL);
    }

    ui->actionFSK->setChecked(false);
    ui->actionLoRa_mode->setChecked(true); // if pressed when already checked

	ui->radioButtonRxSingle->setEnabled(true);
	ui->radioButtonCAD->setEnabled(true);
}

void MainWindow::on_actionFSK_triggered()
{
    ui->stackedWidget->setCurrentIndex(1);
    ui->stackedWidgetIrqFlags->setCurrentIndex(1);

    SX127x.RegOpMode.octet = radio_read(REG_OPMODE);
    if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
        if (SX127x.RegOpMode.fsk_bits.Mode != RF_OPMODE_SLEEP) {
            SX127x.RegOpMode.fsk_bits.Mode = RF_OPMODE_SLEEP;
            radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
        }
        SX127x.RegOpMode.fsk_bits.LongRangeMode = 0;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
        stop_(STOP_ALL);
    }

    ui->actionLoRa_mode->setChecked(false);
    ui->actionFSK->setChecked(true); // if pressed when already checked

	ui->radioButtonRxSingle->setEnabled(false);
	ui->radioButtonCAD->setEnabled(false);
}

void MainWindow::on_actionCommon_triggered()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void MainWindow::on_radioButtonSleep_clicked()
{
    set_opmode(RF_OPMODE_SLEEP);
    stop_(STOP_TX_RUN | STOP_RX_RUN);
}

void MainWindow::on_radioButtonStandby_clicked()
{
    set_opmode(RF_OPMODE_STANDBY);
    stop_(STOP_TX_RUN | STOP_RX_RUN);
}

void MainWindow::on_radioButtonFSRX_clicked()
{
    set_opmode(RF_OPMODE_SYNTHESIZER_RX);
    stop_(STOP_TX_RUN | STOP_RX_RUN);
}

void MainWindow::on_radioButtonFSTX_clicked()
{
    set_opmode(RF_OPMODE_SYNTHESIZER_TX);
    stop_(STOP_RX_RUN);
}

void MainWindow::on_radioButtonRX_clicked()
{
    stop_(STOP_TX_RUN);
    set_opmode(RF_OPMODE_RECEIVER);
}

void MainWindow::on_radioButtonTX_clicked()
{
    stop_(STOP_RX_RUN);
    set_opmode(RF_OPMODE_TRANSMITTER);
}

void MainWindow::on_radioButtonCAD_clicked()
{
    stop_(STOP_TX_RUN | STOP_RX_RUN);
	set_opmode(RF_OPMODE_CAD);
}

void MainWindow::on_radioButtonRxSingle_clicked()
{
    stop_(STOP_TX_RUN | STOP_RX_RUN);
	set_opmode(RF_OPMODE_RECEIVER_SINGLE);
}

void MainWindow::on_listWidgetDIO5FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping2.bits.Dio5Mapping = index.row();
    radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
}

void MainWindow::on_listWidgetDIO4FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping2.bits.Dio4Mapping = index.row();
    radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
}

void MainWindow::on_listWidgetDIO3FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio3Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void MainWindow::on_listWidgetDIO2FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio2Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void MainWindow::on_listWidgetDIO1FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio1Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void MainWindow::on_listWidgetDIO0FSK_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio0Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void MainWindow::regen_fsk_dio_lists()
{
    ui->listWidgetDIO5FSK->clear();
    ui->listWidgetDIO4FSK->clear();
    ui->listWidgetDIO3FSK->clear();
    ui->listWidgetDIO2FSK->clear();
    ui->listWidgetDIO1FSK->clear();
    ui->listWidgetDIO0FSK->clear();

    if (SX127xFSK->RegPktConfig2.bits.DataModePacket) {
        ui->listWidgetDIO5FSK->addItem("ClkOut");
        ui->listWidgetDIO5FSK->addItem("PllLock");
        ui->listWidgetDIO5FSK->addItem("Data");
        ui->listWidgetDIO5FSK->addItem("ModeReady");

        ui->listWidgetDIO4FSK->addItem("TempChange/LowBat");
        ui->listWidgetDIO4FSK->addItem("PllLock");
        ui->listWidgetDIO4FSK->addItem("Timeout");
        ui->listWidgetDIO4FSK->addItem("Rssi/PreambleDetect");

        ui->listWidgetDIO3FSK->addItem("FifoEmpty");
        ui->listWidgetDIO3FSK->addItem("TxReady");
        ui->listWidgetDIO3FSK->addItem("FifoEmpty");
        ui->listWidgetDIO3FSK->addItem("FifoEmpty");

        ui->listWidgetDIO2FSK->addItem("FifoFull");
        ui->listWidgetDIO2FSK->addItem("RxReady");
        ui->listWidgetDIO2FSK->addItem("FifoFull/Timeout");
        ui->listWidgetDIO2FSK->addItem("FifoFull/SyncAddress");

        ui->listWidgetDIO1FSK->addItem("FifoLevel");
        ui->listWidgetDIO1FSK->addItem("FifoEmpty");
        ui->listWidgetDIO1FSK->addItem("FifoFull");
        ui->listWidgetDIO1FSK->addItem("");

        ui->listWidgetDIO0FSK->addItem("PayloadReady / PacketSent");
        ui->listWidgetDIO0FSK->addItem("CrcOk");
        ui->listWidgetDIO0FSK->addItem("");
        ui->listWidgetDIO0FSK->addItem("TempChange/LowBat");
    } else { // continuous:
        ui->listWidgetDIO5FSK->addItem("ClkOut");
        ui->listWidgetDIO5FSK->addItem("PllLock");
        ui->listWidgetDIO5FSK->addItem("Rssi/PreambleDetect");
        ui->listWidgetDIO5FSK->addItem("ModeReady");

        ui->listWidgetDIO4FSK->addItem("TempChange/LowBat");
        ui->listWidgetDIO4FSK->addItem("PllLock");
        ui->listWidgetDIO4FSK->addItem("Timeout");
        ui->listWidgetDIO4FSK->addItem("ModeReady");

        ui->listWidgetDIO3FSK->addItem("Timeout");
        ui->listWidgetDIO3FSK->addItem("Rssi/PreambleDetect");
        ui->listWidgetDIO3FSK->addItem("");
        ui->listWidgetDIO3FSK->addItem("TempChange/LowBat");

        ui->listWidgetDIO2FSK->addItem("Data");
        ui->listWidgetDIO2FSK->addItem("Data");
        ui->listWidgetDIO2FSK->addItem("Data");
        ui->listWidgetDIO2FSK->addItem("Data");

        ui->listWidgetDIO1FSK->addItem("Dclk");
        ui->listWidgetDIO1FSK->addItem("Rssi/PreambleDetect");
        ui->listWidgetDIO1FSK->addItem("");
        ui->listWidgetDIO1FSK->addItem("");

        ui->listWidgetDIO0FSK->addItem("SyncAddress / TxReady");
        ui->listWidgetDIO0FSK->addItem("Rssi/PreambleDetect");
        ui->listWidgetDIO0FSK->addItem("RxReady / TxReady");

    }

}

uint32_t MainWindow::get_tx_fdev_hz(void)
{
    uint16_t fdev = radio_read_u16(REG_FSK_FDEVMSB);
    return fdev * FRF_HZ;
}

uint32_t MainWindow::get_bitrate(void)
{
    uint16_t br = radio_read_u16(REG_FSK_BITRATEMSB);
    if (br == 0)
        return 0;
    else {
        uint8_t frac;
		if (sx1276)
			frac = radio_read(REG_BITRATEFRAC_SX1276);
		else
			frac = radio_read(REG_BITRATEFRAC_SX1272);

        if (frac > 0) {
            return cfg.xtal_hz / (br+(frac/16.0));
        } else
            return (uint32_t)(cfg.xtal_hz / (float)br);
    }
}

void MainWindow::read_fsk_dio()
{
    ui->listWidgetDIO5FSK->setCurrentRow(SX127x.RegDioMapping2.bits.Dio5Mapping);
    ui->listWidgetDIO4FSK->setCurrentRow(SX127x.RegDioMapping2.bits.Dio4Mapping);

    ui->listWidgetDIO3FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio3Mapping);
    ui->listWidgetDIO2FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio2Mapping);
    ui->listWidgetDIO1FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio1Mapping);
    ui->listWidgetDIO0FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio0Mapping);
}



void MainWindow::getRadioStatus_fsk()
{
    int8_t s8;
    char str[48];
	int hz;

    SX127xFSK->RegRxConfig.octet = radio_read(REG_FSK_RXCONFIG); // at 0x0d
    SX127xFSK->RegRssiConfig.octet = radio_read(REG_FSK_RSSICONFIG);    // at 0x0e
    SX127xFSK->RegRssiThresh = radio_read(REG_FSK_RSSITHRESH);  // at 0x10
    SX127xFSK->RegAfcFei.octet = radio_read(REG_FSK_AFCFEI); // at 0x1a
    SX127xFSK->RegPreambleDetect.octet = radio_read(REG_FSK_PREAMBLEDETECT); // at 0x1f
    SX127xFSK->RegPreambleSize = radio_read_u16(REG_FSK_PREAMBLEMSB); // at 0x25,0x26
    SX127xFSK->RegSyncConfig.octet = radio_read(REG_FSK_SYNCCONFIG); // at 0x27
    SX127xFSK->RegPktConfig1.octet = radio_read(REG_FSK_PACKETCONFIG1); // at 0x30
    SX127xFSK->RegPktConfig2.word = radio_read_u16(REG_FSK_PACKETCONFIG2); // at 0x31->0x32
    SX127xFSK->RegNodeAdrs  = radio_read(REG_FSK_NODEADRS); // at 0x33
    SX127xFSK->RegBroadcastAdrs  = radio_read(REG_FSK_BROADCASTADRS);    // at 0x34
    SX127xFSK->RegFifoThreshold.octet = radio_read(REG_FSK_FIFOTHRESH); // at 0x35

    ui->comboBoxDataMode->setCurrentIndex(SX127xFSK->RegPktConfig2.bits.DataModePacket);

    if (list_DataModePacket != SX127xFSK->RegPktConfig2.bits.DataModePacket) {
        regen_fsk_dio_lists();
        fsk_enable_packet_mode(SX127xFSK->RegPktConfig2.bits.DataModePacket);
    }

    list_DataModePacket = SX127xFSK->RegPktConfig2.bits.DataModePacket;

    ui->spinBoxRegPayloadLength->setValue(SX127xFSK->RegPktConfig2.bits.PayloadLength);

    read_fsk_dio();


	hz = get_rx_bw_hz(REG_FSK_RXBW);
	for (s8 = 0; s8 < num_bws; s8++) {
		float diff = fabs(bws[s8] - hz);
		if (diff < 0.1) {
			ui->comboBoxRXBW->setCurrentIndex(s8);
			break;
		}
	}

	hz = get_rx_bw_hz(REG_FSK_AFCBW);
	for (s8 = 0; s8 < num_bws; s8++) {
		float diff = fabs(bws[s8] - hz);
		if (diff < 0.1) {
			ui->comboBoxAfcBW->setCurrentIndex(s8);
			break;
		}
	}

    ui->spinBoxBitrate->setValue(get_bitrate());
    if (sx1276)
		ui->spinBoxFracBR->setValue(radio_read(REG_BITRATEFRAC_SX1276));
	else
		ui->spinBoxFracBR->setValue(radio_read(REG_BITRATEFRAC_SX1272));

    ui->spinBoxFdev->setValue(get_tx_fdev_hz());

    ui->comboBoxOOK->setCurrentIndex(SX127x.RegOpMode.fsk_bits.ModulationType);

    if (sx1276)
        ui->comboBoxShaping->setCurrentIndex(SX127x.RegPaRamp.bits.ModulationShaping_sx1276);
    else
        ui->comboBoxShaping->setCurrentIndex(SX127x.RegOpMode.fsk_bits.ModulationShaping_sx1272);


    ui->checkBoxAfcAutoClearOn->setChecked(SX127xFSK->RegAfcFei.bits.AfcAutoClearOn);

    ui->checkBoxPreambleDetEnable->setChecked(SX127xFSK->RegPreambleDetect.bits.PreambleDetectorOn);
    ui->comboBoxPreambleDetSize->setCurrentIndex(SX127xFSK->RegPreambleDetect.bits.PreambleDetectorSize);
    ui->spinBoxPreambleErrTol->setValue(SX127xFSK->RegPreambleDetect.bits.PreambleDetectorTol);

    ui->checkBoxAfcAutoOn->setChecked(SX127xFSK->RegRxConfig.bits.AfcAutoOn);
    ui->checkBoxRestartRxOnCollision->setChecked(SX127xFSK->RegRxConfig.bits.RestartRxOnCollision);
    switch (SX127xFSK->RegRxConfig.bits.RxTrigger) {
        case 0: ui->comboBoxRxTrigger->setCurrentIndex(0); break; // none
        case 1: ui->comboBoxRxTrigger->setCurrentIndex(1); break; // rssi
        case 6: ui->comboBoxRxTrigger->setCurrentIndex(2); break; // preamble
        case 7: ui->comboBoxRxTrigger->setCurrentIndex(3); break; // both
    }

    ui->checkBoxAgcAutoOn->setChecked(SX127xFSK->RegRxConfig.bits.AgcAutoOn);

    if (SX127xFSK->RegPktConfig2.bits.IoHomeOn) {
        ui->spinBoxFSKSyncSize->setValue(SX127xFSK->RegSyncConfig.bits.SyncSize);
        ui->spinBoxFSKSyncSize->setMinimum(0);
        ui->spinBoxFSKSyncSize->setMaximum(7);
    } else {
        ui->spinBoxFSKSyncSize->setValue(SX127xFSK->RegSyncConfig.bits.SyncSize+1);
        ui->spinBoxFSKSyncSize->setMinimum(1);
        ui->spinBoxFSKSyncSize->setMaximum(8);
    }

    ui->checkBoxFSKSyncOn->setChecked(SX127xFSK->RegSyncConfig.bits.SyncOn);
    ui->comboBoxFSKPreamblePolarity->setCurrentIndex(SX127xFSK->RegSyncConfig.bits.PreamblePolarity);
    ui->comboBoxFSKAutoRestartRxMode->setCurrentIndex(SX127xFSK->RegSyncConfig.bits.AutoRestartRxMode);
    ui->comboBoxFSKFIfoFillCondition->setCurrentIndex(SX127xFSK->RegSyncConfig.bits.FifoFillCondition);

    read_fsk_sync_word();

    ui->comboBoxFSKPktFormat->setCurrentIndex(SX127xFSK->RegPktConfig1.bits.PacketFormatVariable);
    ui->comboBoxFSKCrcWhiteningType->setCurrentIndex(SX127xFSK->RegPktConfig1.bits.CrCWhiteningType);
    ui->comboBoxFSKDcFree->setCurrentIndex(SX127xFSK->RegPktConfig1.bits.DcFree);
    ui->comboBoxFSKAdrsFilt->setCurrentIndex(SX127xFSK->RegPktConfig1.bits.AddressFiltering);
    AddressFilteringChanged();

    ui->labelFSKPktLen->setEnabled(SX127xFSK->RegPktConfig1.bits.PacketFormatVariable);
    ui->labelFSKPktLenHeader->setEnabled(SX127xFSK->RegPktConfig1.bits.PacketFormatVariable);

    ui->checkBoxFSKCrCOn->setChecked(SX127xFSK->RegPktConfig1.bits.CrcOn);
    ui->checkBoxFSKCrcAutoClearOff->setChecked(SX127xFSK->RegPktConfig1.bits.CrcAutoClearOff);

    ui->comboBoxTxStartCond->setCurrentIndex(SX127xFSK->RegFifoThreshold.bits.TxStartCondition);
    ui->spinBoxFSKFifoThreshold->setValue(SX127xFSK->RegFifoThreshold.bits.FifoThreshold);


    ui->spinBoxFSKPreambleSize->setValue(SX127xFSK->RegPreambleSize);

    sprintf(str, "0x%02x", SX127xFSK->RegNodeAdrs);
    ui->lineEditFSKNodeAdrs->setText(str);

    sprintf(str, "0x%02x", SX127xFSK->RegBroadcastAdrs);
    ui->lineEditFSKBcastAdrs->setText(str);

    update_labelFSKPreamble();

    ui->doubleSpinBoxRssiThreshold->setValue(0 - (SX127xFSK->RegRssiThresh / 2.0));

    ui->comboBoxRssiSmoothing->setCurrentIndex(SX127xFSK->RegRssiConfig.bits.RssiSmoothing);
    s8 = SX127xFSK->RegRssiConfig.bits.RssiOffset;
    if (s8 > 15)
        s8 -= 32;
    ui->spinBoxRssiOffset->setValue(s8);
}

void MainWindow::update_dBmout_spinbox()
{
    if (SX127x.RegPaConfig.bits.PaSelect) {
        // PA_BOOST: pout = 2 + OutputPower
        ui->spinBoxdBmout->setMinimum(2);
        if (SX127x.RegPaDac.bits.prog_txdac < 7)
            ui->spinBoxdBmout->setMaximum(17);
        else
            ui->spinBoxdBmout->setMaximum(20);
        ui->spinBoxdBmout->setValue(SX127x.RegPaConfig.bits.OutputPower + 2);
    } else {
        // RFO: pout = -1 + OutputPower
        ui->spinBoxdBmout->setMinimum(-1);
        ui->spinBoxdBmout->setMaximum(13);
        ui->spinBoxdBmout->setValue(SX127x.RegPaConfig.bits.OutputPower - 1);
    }
}

void MainWindow::getRadioStatus()
{
	int imax;
	RegLowBat_t reg_lowbat;
	RegImageCal_t reg_imagecal;

    SX127x.RegOpMode.octet = radio_read(REG_OPMODE);

	showOpMode();

	SX127x.RegPaRamp.octet = radio_read(REG_PARAMP); // at 0x0a
	SX127x.RegOcp.octet = radio_read(REG_OCP);	// at 0x0b
	SX127x.RegLna.octet = radio_read(REG_LNA);		// 0x0c

    SX127x.RegDioMapping1.octet = radio_read(REG_DIOMAPPING1);	// at 0x40
    SX127x.RegDioMapping2.octet = radio_read(REG_DIOMAPPING2);	// at 0x41

    ui->actionFSK->setChecked(!SX127x.RegOpMode.fsk_bits.LongRangeMode);
    ui->actionLoRa_mode->setChecked(SX127x.RegOpMode.fsk_bits.LongRangeMode);

	ui->radioButtonRxSingle->setEnabled(SX127x.RegOpMode.fsk_bits.LongRangeMode);
	ui->radioButtonCAD->setEnabled(SX127x.RegOpMode.fsk_bits.LongRangeMode);

    if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
        if (ui->stackedWidget->currentIndex() == 1)	// if in FSK
			ui->stackedWidget->setCurrentIndex(0);

        ui->stackedWidgetIrqFlags->setCurrentIndex(0);

        ui->widget_lora->getRadioStatus_lora();
    } else {  // fsk:
        if (ui->stackedWidget->currentIndex() == 0)	// if in lora
			ui->stackedWidget->setCurrentIndex(1);

        ui->stackedWidgetIrqFlags->setCurrentIndex(1);

        getRadioStatus_fsk();
    }

    // ****************** common read:
    ui->doubleSpinBoxFrf->setValue(spi.getMHz());

    SX127x.RegPaDac.octet = radio_read(REG_PADAC);

    SX127x.RegPaConfig.octet = radio_read(REG_PACONFIG);
    ui->comboBoxTXpin->setCurrentIndex(SX127x.RegPaConfig.bits.PaSelect);
    update_dBmout_spinbox();

    ui->comboBoxMapPreambleDetect->setCurrentIndex(SX127x.RegDioMapping2.bits.MapPreambleDetect);

    SX127x.RegPll.octet = radio_read(REG_PLL);
    ui->comboBoxTXpllBW->setCurrentIndex(SX127x.RegPll.bits.pllBw);

	ui->comboBoxLNAboostHF->setCurrentIndex(SX127x.RegLna.bits.lnaBoostHF);

	if (sx1276) {
		ui->comboBoxLNAboostLF->setCurrentIndex(SX127x.RegLna.bits.lnaBoostLF);

		SX127x.RegPll_sx1276.octet = radio_read(REG_PLL_SX1276);
		if (SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276)
			ui->comboBoxPLLbwLF->setCurrentIndex(SX127x.RegPll_sx1276.bits.pllBw);
		else
			ui->comboBoxPLLbwHF->setCurrentIndex(SX127x.RegPll_sx1276.bits.pllBw);

		SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276 ^= 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

		SX127x.RegPll_sx1276.octet = radio_read(REG_PLL_SX1276);
		if (SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276)
			ui->comboBoxPLLbwLF->setCurrentIndex(SX127x.RegPll_sx1276.bits.pllBw);
		else
			ui->comboBoxPLLbwHF->setCurrentIndex(SX127x.RegPll_sx1276.bits.pllBw);

	} else {
		ui->checkBoxLowPnTxPllOff->setChecked(SX127x.RegPaRamp.bits.LowPnTxPllOff);
		sx1272_show_pllbw();
	}

	ui->comboBoxPAramp->setCurrentIndex(SX127x.RegPaRamp.bits.PaRamp);

	ui->checkBoxOcpOn->setChecked(SX127x.RegOcp.bits.OcpOn);

	if (SX127x.RegOcp.bits.OcpTrim < 16) {
		imax = 45 + (5*SX127x.RegOcp.bits.OcpTrim);
	} else if (SX127x.RegOcp.bits.OcpTrim < 28) {
		imax = (10*SX127x.RegOcp.bits.OcpTrim) - 30;
	} else
		imax = 240;
	ui->spinBoxOCPtrim->setValue(imax);

	/************** common registers in FSK address space... *****************/
	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_lowbat.octet = radio_read(REG_FSK_LOWBAT);
	ui->checkBoxLowBatOn->setChecked(reg_lowbat.bits.LowBatOn);
	ui->comboBoxLowBatTrim->setCurrentIndex(reg_lowbat.bits.LowBatTrim);

	reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
	ui->checkBoxAutoImageCalOn->setChecked(reg_imagecal.bits.AutoImageCalOn);
	ui->checkBoxTempMonitorOff->setChecked(reg_imagecal.bits.TempMonitorOff);
	ui->comboBoxTempThreshold->setCurrentIndex(reg_imagecal.bits.TempThreshold);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
	/************** ...common registers in FSK address space *****************/

	if (ui->tabWidgetCommon->currentIndex() == 1)
		monitor_temperature();

}

void MainWindow::on_doubleSpinBoxFrf_valueChanged(double arg1)
{
    setMHz(arg1);
}

void MainWindow::fsk_enable_packet_mode(bool en)
{
    ui->plainTextEdit_payload_ascii->setEnabled(en);
    ui->plainTextEdit_payload_hex->setEnabled(en);
    ui->pushButtonPktTXRun->setEnabled(en);
    ui->pushButtonPktRXRun->setEnabled(en);
}

void MainWindow::on_comboBoxDataMode_currentIndexChanged(int index)
{
    if (SX127xFSK->RegPktConfig2.bits.DataModePacket != index) {
        if (index == 0) // going to continuous mode
            stop_(STOP_TX_RUN | STOP_RX_RUN);
    }
    SX127xFSK->RegPktConfig2.bits.DataModePacket = index;
    radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

    regen_fsk_dio_lists();
    read_fsk_dio();

    fsk_enable_packet_mode(SX127xFSK->RegPktConfig2.bits.DataModePacket);
}

void MainWindow::on_comboBoxTXpin_currentIndexChanged(int index)
{
    SX127x.RegPaConfig.bits.PaSelect = index;
    radio_write(REG_PACONFIG, SX127x.RegPaConfig.octet);

    update_dBmout_spinbox();
}

void MainWindow::on_spinBoxdBmout_valueChanged(int arg1)
{
    if (SX127x.RegPaConfig.bits.PaSelect) {
        // PA_BOOST: pout = 2 + OutputPower
        SX127x.RegPaConfig.bits.OutputPower = arg1 - 2;
    } else {
        // RFO: pout = -1 + OutputPower
        SX127x.RegPaConfig.bits.OutputPower = arg1 + 1;
    }
    radio_write(REG_PACONFIG, SX127x.RegPaConfig.octet);
}

void MainWindow::on_checkBoxPADAC_clicked(bool checked)
{
    if (checked)
        SX127x.RegPaDac.bits.prog_txdac = 7; // +20dBm PA_BOOST
    else
        SX127x.RegPaDac.bits.prog_txdac = 4; // +17dBm PA_BOOST (default)
    radio_write(REG_PADAC, SX127x.RegPaDac.octet);
}

void MainWindow::on_actionHW_reset_triggered()
{
    spi.hw_reset();
}

void MainWindow::on_actionRead_all_triggered()
{
    getRadioStatus();
}

uint32_t cnt;

void MainWindow::get_irq_flags()
{
    char str[32];
    if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {

        SX127xLoRa->RegIrqFlags.octet = radio_read(REG_LR_IRQFLAGS);
		show_lora_irqflags();
    } else {
        RegIrqFlags1_t RegIrqFlags1;
        RegIrqFlags2_t RegIrqFlags2;

        RegIrqFlags1.octet = radio_read(REG_FSK_IRQFLAGS1);
        ui->checkBoxModeReady->setChecked(RegIrqFlags1.bits.ModeReady);
        ui->checkBoxRxReady->setChecked(RegIrqFlags1.bits.RxReady);
        ui->checkBoxTxReady->setChecked(RegIrqFlags1.bits.TxReady);
        ui->checkBoxPllLock->setChecked(RegIrqFlags1.bits.PllLock);
        ui->checkBoxRssi->setChecked(RegIrqFlags1.bits.Rssi);
        ui->checkBoxTimeout->setChecked(RegIrqFlags1.bits.Timeout);
        ui->checkBoxPreambleDetect->setChecked(RegIrqFlags1.bits.PreambleDetect);
        ui->checkBoxSyncAddressMatch->setChecked(RegIrqFlags1.bits.SyncAddressMatch);

        RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
        ui->checkBoxFifoFull->setChecked(RegIrqFlags2.bits.FifoFull);
        ui->checkBoxFifoEmpty->setChecked(RegIrqFlags2.bits.FifoEmpty);
        ui->checkBoxFifoLevel->setChecked(RegIrqFlags2.bits.FifoLevel);
        ui->checkBoxFifoOverrun->setChecked(RegIrqFlags2.bits.FifoOverrun);
        ui->checkBoxPacketSent->setChecked(RegIrqFlags2.bits.PacketSent);
        ui->checkBoxPayloadReady->setChecked(RegIrqFlags2.bits.PayloadReady);
        ui->checkBoxCrcOk->setChecked(RegIrqFlags2.bits.CrcOk);
        ui->checkBoxLowBat->setChecked(RegIrqFlags2.bits.LowBat);

        if (ui->tabWidgetFSK->currentIndex() == 1) { // if fsk receiver tab showing
            SX127xFSK->RegFei = radio_read_u16(REG_FSK_FEIMSB);
            sprintf(str, "%.3fKHz", SX127xFSK->RegFei * FRF_KHZ);
            ui->labelFEIValue->setText(str);

            SX127xFSK->RegAfc = radio_read_u16(REG_FSK_AFCMSB);
            sprintf(str, "%.3fKHz", SX127xFSK->RegAfc * FRF_KHZ);
            ui->labelAfcValue->setText(str);

            if (SX127x.RegOpMode.fsk_bits.Mode == RF_OPMODE_RECEIVER) {
                sprintf(str, "-%.1fdBm", radio_read(REG_FSK_RSSIVALUE) / 2.0);
                ui->labelRssiValue->setText(str);
            }
        }

    } // ...!LongRangeMode
}

void MainWindow::setup_FifoLevel(edge_e edge)
{
    if (edge == NO_EDGE) {
        flags.flow_en = 0;
    } else {
        if (dio1_edge != edge) {
            set_dio1_edge(edge);
            if (edge == RISING_EDGE) {   // rising edge used for reception
                fprintf(stderr, "for RISING_EDGE: rf_buf_len=unknown");
                if (!SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
                    fprintf(stderr, ", (fixed) remaining=%d", remaining_);
                }
                fprintf(stderr, "\n");
            }
        }
        if (SX127x.RegDioMapping1.bits.Dio1Mapping != 0) {
            SX127x.RegDioMapping1.bits.Dio1Mapping = 0;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
            ui->listWidgetDIO1FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio1Mapping);
        }
        if (SX127x.RegDioMapping1.bits.Dio3Mapping == 1) { // dio3 to FifoEmpty
            SX127x.RegDioMapping1.bits.Dio3Mapping = 0;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
            ui->listWidgetDIO3FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio3Mapping);
        }
        flags.flow_en = 1;
    } // .. some edge requested
}

void MainWindow::start_tx()
{
	int done;

    if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
		done = ui->widget_lora->_lora_start_tx();
    } else { // FSK...
		done = _fsk_start_tx();
    }

	if (done)
		ui->pushButtonPktTXRun->setChecked(false);
	else {
		char str[32];
        sprintf(str, "%d", ++pkt_count);
        ui->labelPktCnt->setText(str);
	}
}

int MainWindow::_fsk_start_tx()
{
    RegIrqFlags2_t RegIrqFlags2;
	int ret = 0;

    if (pkt_count == total_tx_pkts) {
        killTimer(pkt_tid);
		ret = 1;
    } else { // transmit a packet (start pkt tx)
		// if starting tx on fifoLevel
		if (SX127xFSK->RegFifoThreshold.bits.TxStartCondition == 0) {
			if (SX127xFSK->RegFifoThreshold.bits.FifoThreshold > rf_buf_len) {
				SX127xFSK->RegFifoThreshold.bits.FifoThreshold = rf_buf_len;
				radio_write(REG_FSK_FIFOTHRESH, SX127xFSK->RegFifoThreshold.octet);
				ui->spinBoxFSKFifoThreshold->setValue(SX127xFSK->RegFifoThreshold.bits.FifoThreshold);
				SX127xFSK->RegFifoThreshold.octet = radio_read(REG_FSK_FIFOTHRESH);
			}
		}

		if (tx_busy) {
			fprintf(stderr, "tx_busy %d\n", pkt_tid);
            return ret;
		}
        RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
        if (RegIrqFlags2.bits.FifoEmpty) {
            if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
                if (rf_buf_len > FSK_FIFO_SIZE) {
                    fprintf(stderr, "var-oversized %d\n", rf_buf_len);
                    if (rf_buf_len > 255) {
                        fprintf(stderr, "fail: 255 max varlen (%d)\n", rf_buf_len);
                        return ret;
                    }
                    setup_FifoLevel(FALLING_EDGE);
                    radio_fsk_write_fifo(rf_buf, FSK_FIFO_SIZE, true);
                    remaining_ = rf_buf_len - FSK_FIFO_SIZE;
                } else {
                    setup_FifoLevel(NO_EDGE); // disable
                    radio_fsk_write_fifo(rf_buf, rf_buf_len, true);
                    remaining_ = 0; // all was sent
                }
            } else { // fixed-length pkt format...
                rf_buf_len = SX127xFSK->RegPktConfig2.bits.PayloadLength;
                if (SX127xFSK->RegPktConfig2.bits.PayloadLength > FSK_FIFO_SIZE) {
                    setup_FifoLevel(FALLING_EDGE);
                    radio_fsk_write_fifo(rf_buf, FSK_FIFO_SIZE, true);
                    remaining_ = SX127xFSK->RegPktConfig2.bits.PayloadLength - FSK_FIFO_SIZE;
                } else {
					fprintf(stderr, "fsk fixed tx all %d\n", SX127xFSK->RegPktConfig2.bits.PayloadLength);
                    setup_FifoLevel(NO_EDGE); // disable
                    radio_fsk_write_fifo(rf_buf, SX127xFSK->RegPktConfig2.bits.PayloadLength, true);
                }
            }
        } else {
            int ms;
            fprintf(stderr, "fsk_start_tx: !FifoEmpty %02x\n", RegIrqFlags2.octet);
            killTimer(pkt_tid);

			/* debug: show whats in FIFO */
			do {
				uint8_t o = radio_read(REG_FIFO);
				fprintf(stderr, "in fifo: %02x (opmode:%02x) irqflags2=0x%02x\n", o, radio_read(REG_OPMODE), RegIrqFlags2.octet);
				RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
			} while (!RegIrqFlags2.bits.FifoEmpty);

            ms = ui->spinBoxTxInterval->value() + 10;
            pkt_tid = startTimer(ms);
            ui->spinBoxTxInterval->setValue(ms);
            return ret;
        }


        set_opmode(RF_OPMODE_TRANSMITTER);
		tx_busy = 1;
    }

	return ret;
}

void MainWindow::monitor_temperature()
{
	int8_t stemp;
	char str[16];

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	stemp = radio_read(REG_FSK_TEMP) + 36;
	sprintf(str, "%dC", stemp);
	ui->labelTemp->setText(str);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
}


void MainWindow::timerEvent(QTimerEvent* event)
{
    char str[32];
    int tid = event->timerId();

    if (tid == monitor_tid) {
		get_irq_flags();
		if (ui->tabWidgetCommon->currentIndex() == 1)
			monitor_temperature();
    } else if (tid == pkt_tid) {
		start_tx();
    } // ...if pkt_tid
    else if (tid == rx_tid) {
		if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
			ui->widget_lora->lora_rx_timer_event();     
		} else {
			if (flags.new_afc_value) {
				sprintf(str,"%.3f", SX127xFSK->RegAfc * FRF_KHZ);
				ui->labelPktAFCValue->setText(str);
				flags.new_afc_value = 0;
			}
		}

		if (flags.new_rx_pkt) {
			if (rf_buf_len == LENGTH_UNKNOWN) {
				/* for reception error, ensure flags are displayed */
				get_irq_flags();
			} else {
				sprintf(str, "%d", ++pkt_count);
				ui->labelPktCnt->setText(str);
				fill_hex_TextEdit(rf_buf, rf_buf_len);
			}
			if (SX127x.RegOpMode.fsk_bits.LongRangeMode)
				ui->widget_lora->rx_pkt();     
			else
				reset_flow();// prepare for next (manditory for fixed-length)

			flags.new_rx_pkt = 0;   // prepare for next
		}

    } // ...if rx_tid
	else if (tid == diopoll_tid) {
		uint8_t pins;
		read_all_pins(&pins);
		ui->checkBoxDIO0Pin->setChecked(pins & 0x01);
		ui->checkBoxDIO1Pin->setChecked((pins >> 1) & 0x01);
		ui->checkBoxDIO2Pin->setChecked((pins >> 2) & 0x01);
		ui->checkBoxDIO3Pin->setChecked((pins >> 3) & 0x01);
		ui->checkBoxDIO4Pin->setChecked((pins >> 4) & 0x01);
		ui->checkBoxDIO5Pin->setChecked((pins >> 5) & 0x01);
	} else if (tid == startup_tid) {
		/* one-shot */
        killTimer(startup_tid);
		startup();
    } else
        fprintf(stderr, "unknown tid %d\n", tid);

    if (flags.stop_all) {
        stop_(STOP_ALL);
        flags.stop_all = 0;
    }
}

void MainWindow::on_pushButtonPktTXRun_clicked(bool checked)
{
    if (checked) {
        stop_(STOP_RX_RUN);
        pkt_count = 0;
        ui->labelPktCnt->setText("0");
        if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
			if (SX127x.RegDioMapping1.bits.Dio0Mapping != 1) {
				SX127x.RegDioMapping1.bits.Dio0Mapping = 1;	// to TxDone
				radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
            }
            if (ui->widget_lora->is_implicit_header_mode()) {
                rf_buf_len = SX127xLoRa->RegPayloadLength;
                //fprintf(stderr, "implict lora len %d\n", rf_buf_len);
            } else {
                rf_buf_len = ui->plainTextEdit_payload_ascii->document()->toPlainText().size();
                //fprintf(stderr, "explicit tx len %d\n", rf_buf_len);
                ui->widget_lora->set_payload_length(rf_buf_len);
            }
		} else {
            if (SX127x.RegDioMapping1.bits.Dio0Mapping != 0) {
				ui->listWidgetDIO0FSK->setCurrentRow(0); // to PacketSent
				SX127x.RegDioMapping1.bits.Dio0Mapping = 0;
				radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
			}

			if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable)
				rf_buf_len = ui->plainTextEdit_payload_ascii->document()->toPlainText().size();
			else
				rf_buf_len = SX127xFSK->RegPktConfig2.bits.PayloadLength;
		}

        // count to run: spinBoxFSKRepeat
        total_tx_pkts = ui->spinBoxFSKRepeat->value();
        pkt_tid = startTimer(ui->spinBoxTxInterval->value());

		// do tx now, first timer event wont occur until interval
		start_tx();
    } else
        killTimer(pkt_tid);

}

void MainWindow::fsk_set_dio0_rx()
{
    if (SX127xFSK->RegPktConfig1.bits.CrcOn) {
        if (SX127x.RegDioMapping1.bits.Dio0Mapping != 1) { // to CrcOk
            SX127x.RegDioMapping1.bits.Dio0Mapping = 1;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
            ui->listWidgetDIO0FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio0Mapping);
        }
    } else {
        if (SX127x.RegDioMapping1.bits.Dio0Mapping != 0) { // to PayloadReady
            SX127x.RegDioMapping1.bits.Dio0Mapping = 0;
            radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
            ui->listWidgetDIO0FSK->setCurrentRow(SX127x.RegDioMapping1.bits.Dio0Mapping);
        }
    }
}


void MainWindow::on_pushButtonPktRXRun_clicked(bool checked)
{
    if (checked) {
        stop_(STOP_TX_RUN);
        pkt_count = 0;
        ui->labelPktCnt->setText("0");
        if (SX127x.RegOpMode.fsk_bits.Mode == RF_OPMODE_RECEIVER) {
            // was already receiving, restart it
            set_opmode(RF_OPMODE_STANDBY);
            usleep(20000);
        }
        ui->radioButtonRX->setChecked(true);
        set_opmode(RF_OPMODE_RECEIVER);
		if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
			if (SX127x.RegDioMapping1.bits.Dio0Mapping != 0) {
				SX127x.RegDioMapping1.bits.Dio0Mapping = 0;	// to RxDone
				radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
			}
			// set FIfoAddrPtr to FifoRxBaseAdrs
			radio_write(REG_LR_FIFOADDRPTR, radio_read(REG_LR_FIFORXBASEADDR));
		} else {
			reset_flow();
			fsk_set_dio0_rx();
			if (SX127xFSK->RegPktConfig2.bits.PayloadLength > FSK_FIFO_SIZE)
				setup_FifoLevel(RISING_EDGE);
			else
				setup_FifoLevel(NO_EDGE);
		}
		rx_tid = startTimer(20);
    } else {
        ui->radioButtonStandby->setChecked(true);
        set_opmode(RF_OPMODE_STANDBY);
        killTimer(rx_tid);
    }

}

void MainWindow::on_actionMonitor_triggered(bool checked)
{
    if (checked) {
        monitor_tid = startTimer(50);
    } else {
        killTimer(monitor_tid);
    }

}

void MainWindow::stop_(uint8_t f)
{
    if (f & STOP_MONITOR) {
        if (ui->actionMonitor->isChecked()) {
            killTimer(monitor_tid);
            ui->actionMonitor->setChecked(false);
        }
    }

    if (f & STOP_TX_RUN) {
        if (ui->pushButtonPktTXRun->isChecked()) {
            killTimer(pkt_tid);
            ui->pushButtonPktTXRun->setChecked(false);
        }
    }

    if (f & STOP_RX_RUN) {
        if (ui->pushButtonPktRXRun->isChecked()) {
            killTimer(rx_tid);
            ui->pushButtonPktRXRun->setChecked(false);
            //ui->labelPktRssi->setEnabled(false);
        }
        if (SX127x.RegOpMode.fsk_bits.Mode == RF_OPMODE_RECEIVER) {
            set_opmode(RF_OPMODE_STANDBY);
            ui->radioButtonStandby->setChecked(true);
        }
    }
}

void MainWindow::on_spinBoxBitrate_editingFinished()
{
    set_bitrate(ui->spinBoxBitrate->value());
}

void MainWindow::on_spinBoxFdev_editingFinished()
{
    set_fdev(ui->spinBoxFdev->value());
}

void MainWindow::on_comboBoxOOK_activated(int index)
{
    SX127x.RegOpMode.fsk_bits.ModulationType = index;
    fprintf(stderr, "modulation type index:%d\n", index);
    radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
}

void MainWindow::on_comboBoxShaping_activated(int index)
{
    fprintf(stderr, "modulation shaping index:%d\n", index);
    if (sx1276) {
        SX127x.RegPaRamp.bits.ModulationShaping_sx1276	= index;
        radio_write(REG_PARAMP, SX127x.RegPaRamp.octet);
    } else {
        SX127x.RegOpMode.fsk_bits.ModulationShaping_sx1272 = index;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
    }
}

void MainWindow::on_checkBoxAfcAutoOn_clicked(bool checked)
{
    SX127xFSK->RegRxConfig.bits.AfcAutoOn = checked;
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
}

void MainWindow::on_checkBoxAfcAutoClearOn_clicked(bool checked)
{
    SX127xFSK->RegAfcFei.bits.AfcAutoClearOn = checked;
    radio_write(REG_FSK_AFCFEI, SX127xFSK->RegAfcFei.octet);
}

void MainWindow::on_pushButtonAfcClear_clicked()
{
    SX127xFSK->RegAfcFei.bits.AfcClear = 1;
    radio_write(REG_FSK_AFCFEI, SX127xFSK->RegAfcFei.octet);
    SX127xFSK->RegAfcFei.bits.AfcClear = 0;
}

void MainWindow::on_pushButtonAgcStart_clicked()
{
    SX127xFSK->RegAfcFei.bits.AgcStart = 1;
    radio_write(REG_FSK_AFCFEI, SX127xFSK->RegAfcFei.octet);
    SX127xFSK->RegAfcFei.bits.AgcStart = 0;
}

void MainWindow::on_checkBoxPreambleDetEnable_clicked(bool checked)
{
    SX127xFSK->RegPreambleDetect.bits.PreambleDetectorOn = checked;
    radio_write(REG_FSK_PREAMBLEDETECT, SX127xFSK->RegPreambleDetect.octet);
}

void MainWindow::on_comboBoxPreambleDetSize_currentIndexChanged(int index)
{
    SX127xFSK->RegPreambleDetect.bits.PreambleDetectorSize = index;
    radio_write(REG_FSK_PREAMBLEDETECT, SX127xFSK->RegPreambleDetect.octet);
}

void MainWindow::on_spinBoxPreambleErrTol_editingFinished()
{
    SX127xFSK->RegPreambleDetect.bits.PreambleDetectorTol = ui->spinBoxPreambleErrTol->value();
    radio_write(REG_FSK_PREAMBLEDETECT, SX127xFSK->RegPreambleDetect.octet);
}

void MainWindow::fill_hex_TextEdit(uint8_t* buf, int buf_len)
{
    int i;
    char str[6141], *str_ptr;
    QTextCursor hex_cursor = ui->plainTextEdit_payload_hex->textCursor();

    str_ptr = str;
    *str_ptr = 0; // terminate for zero-length
    for (i = 0; i < buf_len; i++) {
        sprintf(str_ptr, "%02x ", *buf);
        str_ptr += 3;
        buf++;
    }

    hex_cursor.beginEditBlock();
    hex_cursor.movePosition(QTextCursor::Start);
    hex_cursor.select(QTextCursor::Document); // entire document
    hex_cursor.insertText(str);
    hex_cursor.endEditBlock();
}

void MainWindow::on_plainTextEdit_payload_ascii_textChanged()
{
    unsigned int i;
    size_t stop;
    char *ascii_data;
    char str[16];

    if (text_changed_busy)
        return;
    text_changed_busy = true;

    QTextDocument* ascii_doc = ui->plainTextEdit_payload_ascii->document();
    QString ascii_qstr = ascii_doc->toPlainText();
    QByteArray ascii_barray = ascii_qstr.toLocal8Bit();

    ascii_data = ascii_barray.data();
    stop = ascii_barray.count();
    if (stop > sizeof(rf_buf)) {
        stop = sizeof(rf_buf);
        fprintf(stderr, "fifo_buf size:%d\n", sizeof(rf_buf));
    }
    rf_buf_len = 0;
    for (i = 0; i < stop; i++) {
        rf_buf[rf_buf_len++] = *ascii_data;
        ascii_data++;
    }

    fill_hex_TextEdit((uint8_t*)ascii_barray.data(), stop);

    if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
        sprintf(str, "%d", stop);
        ui->labelFSKPktLen->setText(str);
    }

    text_changed_busy = false;
}

void MainWindow::on_plainTextEdit_payload_hex_textChanged()
{
    int i;
    char str[512], *str_ptr;
    char *hex_data;

    if (text_changed_busy)
        return;
    text_changed_busy = true;

    QTextDocument* hex_doc = ui->plainTextEdit_payload_hex->document();
    QTextCursor ascii_cursor = ui->plainTextEdit_payload_ascii->textCursor();
    QString hex_qstr = hex_doc->toPlainText();
    QByteArray hex_barray = hex_qstr.toLocal8Bit();

    hex_data = hex_barray.data();
    str_ptr = str;
    rf_buf_len = 0;
    for (i = 0; i < hex_barray.count(); i += 3) {
        int parsed;
        sscanf(hex_data, "%x", &parsed);
        hex_data += 3;
        sprintf(str_ptr, "%c", parsed);
        str_ptr++;
        if (rf_buf_len < (int)sizeof(rf_buf))
            rf_buf[rf_buf_len++] = parsed;
    }
    *str_ptr = 0; // null terminate

    ascii_cursor.beginEditBlock();
    ascii_cursor.movePosition(QTextCursor::Start);
    ascii_cursor.select(QTextCursor::Document); // entire document
    ascii_cursor.insertText(str);
    ascii_cursor.endEditBlock();

    if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
        sprintf(str, "%d", rf_buf_len);
        ui->labelFSKPktLen->setText(str);
    }

    text_changed_busy = false;
}

void MainWindow::on_comboBoxFSKPktFormat_currentIndexChanged(int index)
{
    SX127xFSK->RegPktConfig1.bits.PacketFormatVariable = index;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);

    ui->labelFSKPktLen->setEnabled(SX127xFSK->RegPktConfig1.bits.PacketFormatVariable);
    ui->labelFSKPktLenHeader->setEnabled(SX127xFSK->RegPktConfig1.bits.PacketFormatVariable);
}

void MainWindow::on_comboBoxTxStartCond_currentIndexChanged(int index)
{
    SX127xFSK->RegFifoThreshold.bits.TxStartCondition = index;
    radio_write(REG_FSK_FIFOTHRESH, SX127xFSK->RegFifoThreshold.octet);
}

void MainWindow::on_checkBoxFSKCrCOn_clicked(bool checked)
{
    SX127xFSK->RegPktConfig1.bits.CrcOn = checked;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);
    if (ui->pushButtonPktRXRun->isChecked())
        fsk_set_dio0_rx();
}

void MainWindow::on_spinBoxFSKFifoThreshold_editingFinished()
{
    SX127xFSK->RegFifoThreshold.bits.FifoThreshold = ui->spinBoxFSKFifoThreshold->value();
    radio_write(REG_FSK_FIFOTHRESH, SX127xFSK->RegFifoThreshold.octet);
}

void MainWindow::set_SyncSize(int v)
{
    if (SX127xFSK->RegPktConfig2.bits.IoHomeOn)
        SX127xFSK->RegSyncConfig.bits.SyncSize = v;
    else
        SX127xFSK->RegSyncConfig.bits.SyncSize = v - 1;

    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);
}


void MainWindow::on_spinBoxFSKSyncSize_valueChanged(int arg1)
{
    // this spinbox is single digit, no need for editingFinished()
    set_SyncSize(arg1);

    read_fsk_sync_word();
}

void MainWindow::read_fsk_sync_word()
{
    char str[48], *str_ptr;
    int i, stop;

    stop = ui->spinBoxFSKSyncSize->value();
    str_ptr = str;
    for (i = 0; i < stop; ) {
        SX127xFSK->RegSyncValues[i] = radio_read(REG_FSK_SYNCVALUE1+i);
        sprintf(str_ptr, "%02x", SX127xFSK->RegSyncValues[i]);
        str_ptr += 2;
        if (++i < stop)
            *str_ptr++ = '-';
    }
    *str_ptr = 0;
    ui->lineEditFSKSyncWord->setText(str);
    ui->labelFSKSync->setText(str);
}

void MainWindow::on_lineEditFSKSyncWord_editingFinished()
{
    int oct, i;
    QString qstr = ui->lineEditFSKSyncWord->text();
    QByteArray barray = qstr.toLocal8Bit();
    char *data = barray.data();

    i = 0;
    for (i = 0; *data != 0; i++) {
        if (i > 8)
            break;
        sscanf(data, "%02x", &oct);
        data += 2;
        SX127xFSK->RegSyncValues[i] = oct;
        radio_write(REG_FSK_SYNCVALUE1+i, SX127xFSK->RegSyncValues[i]);
        while (*data != 0 && (*data < '0' || (*data > '9' && *data < 'A') || (*data > 'F' && *data < 'a') || *data > 'f'))
            data++;
    }

    if (i != ui->spinBoxFSKSyncSize->value()) {
        ui->spinBoxFSKSyncSize->setValue(i);
        set_SyncSize(i);
    }


}

void MainWindow::on_spinBoxRegPayloadLength_valueChanged(int arg1)
{
    SX127xFSK->RegPktConfig2.bits.PayloadLength = arg1;
    radio_write_u16(REG_FSK_PACKETCONFIG2, SX127xFSK->RegPktConfig2.word);

    if (ui->pushButtonPktRXRun->isChecked()) {
        if (SX127xFSK->RegPktConfig2.bits.PayloadLength > FSK_FIFO_SIZE)
            setup_FifoLevel(RISING_EDGE);
        else
            setup_FifoLevel(NO_EDGE);
        if (!SX127xFSK->RegPktConfig1.bits.PacketFormatVariable)
            rf_buf_len = arg1;
    }

}

void MainWindow::on_actionOpen_Config_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Config (write to radio)"), "", tr("Files(*.cfg)"));
    cfg.openFile(filename.toLocal8Bit().data());

    getRadioStatus();
}

void MainWindow::on_actionSave_Config_triggered()
{
    cfg.save(NULL); // using cfg.path
}

void MainWindow::on_actionSave_Config_As_triggered()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Config (read from radio)"), "", tr("Files(*.cfg)"));
    cfg.save(filename.toLocal8Bit().data());
}

void MainWindow::on_checkBoxFSKSyncOn_clicked(bool checked)
{
    SX127xFSK->RegSyncConfig.bits.SyncOn = checked;
    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);
}



void MainWindow::on_checkBoxFSKCrcAutoClearOff_clicked(bool checked)
{
    SX127xFSK->RegPktConfig1.bits.CrcAutoClearOff = checked;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);
}

void MainWindow::on_comboBoxFSKAutoRestartRxMode_currentIndexChanged(int index)
{
    SX127xFSK->RegSyncConfig.bits.AutoRestartRxMode = index;
    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);
}

void MainWindow::update_labelFSKPreamble()
{
    int i;
    char c;
    char str[32];
    char *str_ptr = str;
    char *stop_ptr = &str[sizeof(str)-1];

    if (SX127xFSK->RegSyncConfig.bits.PreamblePolarity)
        c = '5';
    else
        c = 'A';

    for (i = 0; i < SX127xFSK->RegPreambleSize; i++) {
         *str_ptr++ = c;
         *str_ptr++ = c;
         if (str_ptr < stop_ptr && i < (SX127xFSK->RegPreambleSize-1))
            *str_ptr++ = '-';
         else
            *str_ptr = 0;
    }

    ui->labelFSKPreamble->setText(str);
}

void MainWindow::on_spinBoxFSKPreambleSize_editingFinished()
{
    SX127xFSK->RegPreambleSize = ui->spinBoxFSKPreambleSize->value();
    radio_write_u16(REG_FSK_PREAMBLEMSB, SX127xFSK->RegPreambleSize);
    update_labelFSKPreamble();
}

void MainWindow::on_comboBoxFSKPreamblePolarity_currentIndexChanged(int index)
{
    SX127xFSK->RegSyncConfig.bits.PreamblePolarity = index;
    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);
    update_labelFSKPreamble();
}

void MainWindow::on_comboBoxFSKFIfoFillCondition_currentIndexChanged(int index)
{
    SX127xFSK->RegSyncConfig.bits.FifoFillCondition = index;
    radio_write(REG_FSK_SYNCCONFIG, SX127xFSK->RegSyncConfig.octet);
}

void MainWindow::on_comboBoxFSKCrcWhiteningType_currentIndexChanged(int index)
{
    SX127xFSK->RegPktConfig1.bits.CrCWhiteningType = index;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);
}

void MainWindow::on_comboBoxFSKDcFree_currentIndexChanged(int index)
{
    SX127xFSK->RegPktConfig1.bits.DcFree = index;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);
}


void MainWindow::AddressFilteringChanged()
{
    switch(SX127xFSK->RegPktConfig1.bits.AddressFiltering) {
        case 0:
            ui->lineEditFSKAddrToSend->setEnabled(false);
            ui->labelFSKNodeAdrsHeader->setEnabled(false);
            ui->lineEditFSKNodeAdrs->setEnabled(false);
            ui->lineEditFSKBcastAdrs->setEnabled(false);
            break;
        case 1:
            ui->lineEditFSKAddrToSend->setEnabled(true);
            ui->labelFSKNodeAdrsHeader->setEnabled(true);
            ui->lineEditFSKNodeAdrs->setEnabled(true);
            ui->lineEditFSKBcastAdrs->setEnabled(false);
            break;
        case 2:
            ui->lineEditFSKAddrToSend->setEnabled(true);
            ui->labelFSKNodeAdrsHeader->setEnabled(true);
            ui->lineEditFSKNodeAdrs->setEnabled(true);
            ui->lineEditFSKBcastAdrs->setEnabled(true);
            break;
    }
}

void MainWindow::on_comboBoxFSKAdrsFilt_currentIndexChanged(int index)
{
    SX127xFSK->RegPktConfig1.bits.AddressFiltering = index;
    radio_write(REG_FSK_PACKETCONFIG1, SX127xFSK->RegPktConfig1.octet);
    AddressFilteringChanged();
}

void MainWindow::on_lineEditFSKNodeAdrs_editingFinished()
{
    char *str = ui->lineEditFSKNodeAdrs->text().toLocal8Bit().data();
    SX127xFSK->RegNodeAdrs = strtol(str, NULL, 0);
    radio_write(REG_FSK_NODEADRS, SX127xFSK->RegNodeAdrs);
}

void MainWindow::on_lineEditFSKBcastAdrs_editingFinished()
{
    char *str = ui->lineEditFSKBcastAdrs->text().toLocal8Bit().data();
    SX127xFSK->RegBroadcastAdrs = strtol(str, NULL, 0);

    radio_write(REG_FSK_BROADCASTADRS, SX127xFSK->RegBroadcastAdrs);
}

void MainWindow::on_lineEditFSKAddrToSend_editingFinished()
{
    char *str = ui->lineEditFSKAddrToSend->text().toLocal8Bit().data();
    NodeAdrs_to_send = strtol(str, NULL, 0);
}

void MainWindow::on_comboBoxMapPreambleDetect_currentIndexChanged(int index)
{
    SX127x.RegDioMapping2.bits.MapPreambleDetect = index;
    radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
}

void MainWindow::on_comboBoxRxTrigger_currentIndexChanged(int index)
{
    switch (index) {
        case 0: SX127xFSK->RegRxConfig.bits.RxTrigger = 0; break; // none
        case 1: SX127xFSK->RegRxConfig.bits.RxTrigger = 1; break; // rssi
        case 2: SX127xFSK->RegRxConfig.bits.RxTrigger = 6; break; // preamble
        case 3: SX127xFSK->RegRxConfig.bits.RxTrigger = 7; break; // both
    }
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
}

void MainWindow::on_checkBoxRestartRxOnCollision_clicked(bool checked)
{
    SX127xFSK->RegRxConfig.bits.RestartRxOnCollision = checked;
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
}

void MainWindow::on_pushButtonRestartRxWithoutPllLock_clicked()
{
    SX127xFSK->RegRxConfig.bits.RestartRxWithoutPllLock = 1;
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
    SX127xFSK->RegRxConfig.bits.RestartRxWithoutPllLock = 0;
}

void MainWindow::on_pushButtonRestartRxWithPllLock_clicked()
{
    SX127xFSK->RegRxConfig.bits.RestartRxWithPllLock = 1;
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
    SX127xFSK->RegRxConfig.bits.RestartRxWithPllLock = 0;
}

void MainWindow::on_checkBoxAgcAutoOn_clicked(bool checked)
{
    SX127xFSK->RegRxConfig.bits.AgcAutoOn = checked;
    radio_write(REG_FSK_RXCONFIG, SX127xFSK->RegRxConfig.octet);
}

void MainWindow::on_doubleSpinBoxRssiThreshold_editingFinished()
{
    SX127xFSK->RegRssiThresh = 0 - (ui->doubleSpinBoxRssiThreshold->value() * 2.0);
    radio_write(REG_FSK_RSSITHRESH, SX127xFSK->RegRssiThresh);
}

void MainWindow::on_comboBoxRssiSmoothing_currentIndexChanged(int index)
{
    SX127xFSK->RegRssiConfig.bits.RssiSmoothing = index;
    radio_write(REG_FSK_RSSICONFIG, SX127xFSK->RegRssiConfig.octet);
}

void MainWindow::on_spinBoxRssiOffset_valueChanged(int arg1)
{
    if (arg1 < 0)
        arg1 += 32;
    SX127xFSK->RegRssiConfig.bits.RssiOffset = arg1;
    radio_write(REG_FSK_RSSICONFIG, SX127xFSK->RegRssiConfig.octet);
}

void MainWindow::on_spinBoxFracBR_editingFinished()
{
	if (sx1276)
		radio_write(REG_BITRATEFRAC_SX1276, ui->spinBoxFracBR->value());
	else
		radio_write(REG_BITRATEFRAC_SX1272, ui->spinBoxFracBR->value());

    ui->spinBoxBitrate->setValue(get_bitrate());
}

void MainWindow::on_comboBoxTXpllBW_currentIndexChanged(int index)
{
    SX127x.RegPll.bits.pllBw = index;
    radio_write(REG_PLL, SX127x.RegPll.octet);
}

void MainWindow::on_comboBoxPAramp_currentIndexChanged(int index)
{
	SX127x.RegPaRamp.bits.PaRamp = index;
    radio_write(REG_PARAMP, SX127x.RegPaRamp.octet);
}

void MainWindow::on_comboBoxLNAboostLF_currentIndexChanged(int index)
{
	SX127x.RegLna.bits.lnaBoostLF = index;
	radio_write(REG_LNA, SX127x.RegLna.octet);
}

void MainWindow::on_comboBoxLNAboostHF_currentIndexChanged(int index)
{
	SX127x.RegLna.bits.lnaBoostHF = index;
	radio_write(REG_LNA, SX127x.RegLna.octet);
}

void MainWindow::on_checkBoxOcpOn_clicked(bool checked)
{
	SX127x.RegOcp.bits.OcpOn = checked;
	radio_write(REG_OCP, SX127x.RegOcp.octet);
}

void MainWindow::on_spinBoxOCPtrim_valueChanged(int arg1)
{
	if (arg1 < 130)
		SX127x.RegOcp.bits.OcpTrim = (arg1-45) / 5;
	else if (arg1 < 240)
		SX127x.RegOcp.bits.OcpTrim = (arg1+30) / 10;

	radio_write(REG_OCP, SX127x.RegOcp.octet);
}

void MainWindow::sx1272_show_pllbw()
{
	if (sx1276) {
		fprintf(stderr, "sx1272_show_pllbw() wrong chip\n");
	} else {
		if (SX127x.RegPaRamp.bits.LowPnTxPllOff) {
			// use reg 0x5c
			SX127x.RegPll.octet = radio_read(REG_PLL);
			ui->comboBoxPLLbwHF->setCurrentIndex(SX127x.RegPll.bits.pllBw);
		} else {
			// use reg 0x5e
			SX127x.RegPllLowPn.octet = radio_read(REG_PLLLOWPN);
			ui->comboBoxPLLbwHF->setCurrentIndex(SX127x.RegPllLowPn.bits.pllBw);
		}
	}
}

void MainWindow::on_checkBoxLowPnTxPllOff_clicked(bool checked)
{
	SX127x.RegPaRamp.bits.LowPnTxPllOff = checked;
    radio_write(REG_PARAMP, SX127x.RegPaRamp.octet);

	sx1272_show_pllbw();
}

void MainWindow::on_comboBoxPLLbwHF_currentIndexChanged(int index)
{
	if (sx1276) {
		if (SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276) {
			SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276 = 0;
			radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
		}
		SX127x.RegPll_sx1276.bits.pllBw = index;
		radio_write(REG_PLL_SX1276, SX127x.RegPll_sx1276.octet);
	} else {
		if (SX127x.RegPaRamp.bits.LowPnTxPllOff) {
			SX127x.RegPll.bits.pllBw = index;
			radio_write(REG_PLL, SX127x.RegPll.octet);
		} else {
			SX127x.RegPllLowPn.bits.pllBw = index;
			radio_write(REG_PLLLOWPN, SX127x.RegPllLowPn.octet);
		}
	}
}

void MainWindow::on_comboBoxPLLbwLF_currentIndexChanged(int index)
{
	if (sx1276) {
		if (!SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276) {
			SX127x.RegOpMode.bits.LowFrequencyModeOn_sx1276 = 1;
			radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
		}
		SX127x.RegPll_sx1276.bits.pllBw = index;
		radio_write(REG_PLL_SX1276, SX127x.RegPll_sx1276.octet);
	} else
		fprintf(stderr, "on_comboBoxPLLbwLF_currentIndexChanged(): sx1272\n");
}

void MainWindow::on_comboBoxRXBW_currentIndexChanged(int index)
{
	if (!init_done)
		return;

    set_rx_dcc_bw_hz(bws[index], 0);
}

void MainWindow::on_comboBoxAfcBW_currentIndexChanged(int index)
{
	if (!init_done)
		return;

    set_rx_dcc_bw_hz(bws[index], 1);
}

void MainWindow::on_checkBoxLowBatOn_clicked(bool checked)
{
	RegLowBat_t reg_lowbat;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_lowbat.octet = radio_read(REG_FSK_LOWBAT);
	reg_lowbat.bits.LowBatOn = checked;
	radio_write(REG_FSK_LOWBAT, reg_lowbat.octet);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
}

void MainWindow::on_comboBoxLowBatTrim_currentIndexChanged(int index)
{
	RegLowBat_t reg_lowbat;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_lowbat.octet = radio_read(REG_FSK_LOWBAT);
	reg_lowbat.bits.LowBatTrim = index;
	radio_write(REG_FSK_LOWBAT, reg_lowbat.octet);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
}

void MainWindow::on_checkBoxAutoImageCalOn_clicked(bool checked)
{
	RegImageCal_t reg_imagecal;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
	reg_imagecal.bits.AutoImageCalOn = checked;
	radio_write(REG_FSK_IMAGECAL, reg_imagecal.octet);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
}

void MainWindow::on_pushButtonImageCalStart_clicked()
{
	RegImageCal_t reg_imagecal;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
	reg_imagecal.bits.ImageCalStart	= 1;
	radio_write(REG_FSK_IMAGECAL, reg_imagecal.octet);

	/* blocking */
	do {
		reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
		fprintf(stderr, "ImageCalRunning:%d\n", reg_imagecal.bits.ImageCalRunning);
	} while (reg_imagecal.bits.ImageCalRunning);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_imagecal.bits.ImageCalStart	= 0;
}

void MainWindow::on_checkBoxTempMonitorOff_clicked(bool checked)
{
	RegImageCal_t reg_imagecal;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
	reg_imagecal.bits.TempMonitorOff = checked;
	radio_write(REG_FSK_IMAGECAL, reg_imagecal.octet);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 0;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

}

void MainWindow::on_comboBoxTempThreshold_currentIndexChanged(int index)
{
	RegImageCal_t reg_imagecal;

	if (SX127x.RegOpMode.bits.LongRangeMode) {
		SX127x.RegOpMode.bits.AccessSharedReg = 1;
		radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}

	reg_imagecal.octet = radio_read(REG_FSK_IMAGECAL);
	reg_imagecal.bits.TempThreshold	= index;
	radio_write(REG_FSK_IMAGECAL, reg_imagecal.octet);

	if (SX127x.RegOpMode.bits.LongRangeMode) {
        SX127x.RegOpMode.bits.AccessSharedReg = 0;
        radio_write(REG_OPMODE, SX127x.RegOpMode.octet);
	}
}

void MainWindow::on_tabWidgetFSK_currentChanged(int index)
{
	if (index == 2) {
        diopoll_tid = startTimer(DIO_POLL_RATE);
	} else {
        killTimer(diopoll_tid);
	}
}

void MainWindow::on_stackedWidget_currentChanged(int arg1)
{
	if (arg1 == 1) {
		// switched to FSK
        regen_fsk_dio_lists();
		killTimer(ui->widget_lora->diopoll_tid);
		if (ui->tabWidgetFSK->currentIndex() == 2)
			diopoll_tid = startTimer(DIO_POLL_RATE);
	} else {
		// switched to LORA
        killTimer(diopoll_tid);
		if (ui->widget_lora->tabWidget->currentIndex() == 1)
			ui->widget_lora->diopoll_tid = ui->widget_lora->startTimer(DIO_POLL_RATE);
	}
}
