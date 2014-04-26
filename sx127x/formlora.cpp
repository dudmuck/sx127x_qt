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
#include "formlora.h"
#include "ui_formlora.h"
#include <stdio.h>
#include "spi.h"

FormLora::FormLora(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FormLora)
{
    ui->setupUi(this);
	init_done = false;

	tabWidget = ui->tabWidget;	// public access
}

FormLora::~FormLora()
{
    delete ui;
}

void FormLora::init()
{
    ui->comboBoxBW->clear();

	if (sx1276) {
		ui->comboBoxBW->addItem("7.8KHz");
		ui->comboBoxBW->addItem("10.4KHz");
		ui->comboBoxBW->addItem("15.6KHz");
		ui->comboBoxBW->addItem("20.8KHz");
		ui->comboBoxBW->addItem("31.25KHz");
		ui->comboBoxBW->addItem("41.7KHz");
		ui->comboBoxBW->addItem("62.5KHz");
		ui->comboBoxBW->addItem("125KHz");
		ui->comboBoxBW->addItem("250KHz");
		ui->comboBoxBW->addItem("500KHz");
	} else {
		ui->comboBoxBW->addItem("125KHz");
		ui->comboBoxBW->addItem("250KHz");
		ui->comboBoxBW->addItem("500KHz");
	}

	init_done = true;
}

void FormLora::rx_pkt(void)
{
	char str[32];
	int snr;

	sprintf(str, "%d", SX127xLoRa->RegHopChannel.bits.FhssPresentChannel);
	ui->lineEditFhssPresentChannel->setText(str);

	sprintf(str, "%d", SX127xLoRa->RxPacketCnt);
	ui->lineEditPacketCnt->setText(str);

	sprintf(str, "0x%02x", SX127xLoRa->RegRxDataStartAddr);
	ui->lineEditRXCurAddr->setText(str);

	if (SX127xLoRa->RegPktSnrValue > 0x80)
		snr = SX127xLoRa->RegPktSnrValue - 256;
	else
		snr = SX127xLoRa->RegPktSnrValue;

	sprintf(str, "%.1fdB", snr / 4.0);
	ui->lineEditPktSNR->setText(str);

	sprintf(str, "%ddBm", SX127xLoRa->RegPktRssiValue - 137);
	ui->lineEditPktRSSI->setText(str);

	//

	ui->checkBoxPllTeimout->setChecked(SX127xLoRa->RegHopChannel.bits.PllTimeout);
	ui->checkBoxRxPayloadCRC->setChecked(SX127xLoRa->RegHopChannel.bits.RxPayloadCrcOn);

	switch (SX127xLoRa->RegModemStatus.bits.RxCodingRate) {
		case 0: strcpy(str, "none"); break;
		case 1: strcpy(str, "4/5"); break;
		case 2: strcpy(str, "4/6"); break;
		case 3: strcpy(str, "4/7"); break;
		case 4: strcpy(str, "4/8"); break;
		default: strcpy(str, "?"); break;
	}
	ui->lineEditRxCR->setText(str);

	sprintf(str, "%d", SX127xLoRa->RegNbRxBytes);
	ui->lineEditRxNb->setText(str);
}

void FormLora::lora_rx_timer_event(void)
{
}

void FormLora::getRadioStatus_lora(void)
{
    SX127xLoRa->RegModemConfig1.octet = radio_read(REG_LR_MODEMCONFIG1);
    SX127xLoRa->RegModemConfig2_timeout.word = radio_read_u16(REG_LR_MODEMCONFIG2);
    SX127xLoRa->RegModemConfig3.octet = radio_read(REG_LR_MODEMCONFIG3);

    ui->comboBoxSF->setCurrentIndex(SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor-6);

    if (sx1276) {
        ui->comboBoxCR->setCurrentIndex(SX127xLoRa->RegModemConfig1.sx1276bits.CodingRate);
		ui->checkBoxCrcOn->setChecked(SX127xLoRa->RegModemConfig2_timeout.sx1276bits.RxPayloadCrcOn);
		ui->comboBoxBW->setCurrentIndex(SX127xLoRa->RegModemConfig1.sx1276bits.Bw);
		ui->checkBoxImplicit->setChecked(SX127xLoRa->RegModemConfig1.sx1276bits.ImplicitHeaderModeOn);
		ui->checkBoxLDRO->setChecked(SX127xLoRa->RegModemConfig3.sx1276bits.LowDataRateOptimize);
		ui->checkBoxLoraAGCAutoOn->setChecked(SX127xLoRa->RegModemConfig3.sx1276bits.AgcAutoOn);
		ui->checkBoxTxContinuousMode->setChecked(SX127xLoRa->RegModemConfig2_timeout.sx1276bits.TxContinuousMode);
    } else {
        ui->comboBoxCR->setCurrentIndex(SX127xLoRa->RegModemConfig1.sx1272bits.CodingRate);
		ui->checkBoxCrcOn->setChecked(SX127xLoRa->RegModemConfig1.sx1272bits.RxPayloadCrcOn);
		ui->comboBoxBW->setCurrentIndex(SX127xLoRa->RegModemConfig1.sx1272bits.Bw);
		ui->checkBoxImplicit->setChecked(SX127xLoRa->RegModemConfig1.sx1272bits.ImplicitHeaderModeOn);
		ui->checkBoxLDRO->setChecked(SX127xLoRa->RegModemConfig1.sx1272bits.LowDataRateOptimize);
		ui->checkBoxLoraAGCAutoOn->setChecked(SX127xLoRa->RegModemConfig2_timeout.sx1272bits.AgcAutoOn);
		ui->checkBoxTxContinuousMode->setChecked(SX127xLoRa->RegModemConfig2_timeout.sx1272bits.TxContinuousMode);
	}

	ui->spinBoxSymbTimeout->setValue(SX127xLoRa->RegModemConfig2_timeout.sx1276bits.SymbTimeout);

	SX127xLoRa->RegPreambleLength = radio_read_u16(REG_LR_PREAMBLEMSB);
	ui->spinBoxPreambleLength->setValue(SX127xLoRa->RegPreambleLength);

	SX127xLoRa->RegPayloadLength = radio_read(REG_LR_PAYLOADLENGTH);
	ui->spinBoxPayloadLength->setValue(SX127xLoRa->RegPayloadLength);

    SX127xLoRa->RegRxMaxPayloadLength = radio_read(REG_LR_RX_MAX_PAYLOADLENGTH);
    ui->spinBoxMaxPayloadLength->setValue(SX127xLoRa->RegRxMaxPayloadLength);

	ui->listWidgetDIO5->setCurrentRow(SX127x.RegDioMapping2.bits.Dio5Mapping);
	ui->listWidgetDIO4->setCurrentRow(SX127x.RegDioMapping2.bits.Dio4Mapping);
	ui->listWidgetDIO3->setCurrentRow(SX127x.RegDioMapping1.bits.Dio3Mapping);
	ui->listWidgetDIO2->setCurrentRow(SX127x.RegDioMapping1.bits.Dio2Mapping);
	ui->listWidgetDIO1->setCurrentRow(SX127x.RegDioMapping1.bits.Dio1Mapping);
    ui->listWidgetDIO0->setCurrentRow(SX127x.RegDioMapping1.bits.Dio0Mapping);

    SX127xLoRa->RegTest31.octet = radio_read(REG_LR_TEST31);

}

int FormLora::_lora_start_tx(void)
{
	int ret = 0;

    if (pkt_count == total_tx_pkts) {
        killTimer(pkt_tid);
		ret = 1;
    } else { // transmit a packet (start pkt tx)
        /*SX127xLoRa->RegPayloadLength = radio_read(REG_LR_PAYLOADLENGTH);
        fprintf(stderr, "lora start tx %d, regpayloadlen:%d\n", rf_buf_len, SX127xLoRa->RegPayloadLength);*/
		radio_lora_write_fifo(rf_buf, rf_buf_len);
		set_opmode(RF_OPMODE_TRANSMITTER);
	}

	return ret;
}

void FormLora::timerEvent(QTimerEvent* event)
{
    int tid = event->timerId();
	if (tid == diopoll_tid) {
		uint8_t pins;
		read_all_pins(&pins);
		ui->checkBoxDio5pin->setChecked((pins >> 5) & 1);
		ui->checkBoxDio4pin->setChecked((pins >> 4) & 1);
		ui->checkBoxDio3pin->setChecked((pins >> 3) & 1);
		ui->checkBoxDio2pin->setChecked((pins >> 2) & 1);
		ui->checkBoxDio1pin->setChecked((pins >> 1) & 1);
		ui->checkBoxDio0pin->setChecked(pins & 1);
	} else
		fprintf(stderr, "FormLora::timerEvent() unknown tid %d\n", tid);
}

void FormLora::on_comboBoxSF_currentIndexChanged(int index)
{
    SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor = index + 6;
    radio_write_u16(REG_LR_MODEMCONFIG2, SX127xLoRa->RegModemConfig2_timeout.word);

    switch (SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor) {
        case 6:
            set_nb_trig_peaks(3);
            break;
        case 7:
            set_nb_trig_peaks(4);
            break;
        default:
            set_nb_trig_peaks(5);
            break;
    }

    if (SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor < 7)
        radio_write(REG_LR_DETECTION_THRESHOLD, 0x0c);
    else
        radio_write(REG_LR_DETECTION_THRESHOLD, 0x0a);


    if (!sx1276) {
        SX127xLoRa->RegModemConfig1.sx1272bits.CodingRate = index;
        if (index > 10 && SX127xLoRa->RegModemConfig1.sx1276bits.Bw == 0)
            SX127xLoRa->RegModemConfig1.sx1272bits.LowDataRateOptimize = 1;
        else
            SX127xLoRa->RegModemConfig1.sx1272bits.LowDataRateOptimize = 0;
        radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
        ui->checkBoxLDRO->setChecked(SX127xLoRa->RegModemConfig1.sx1272bits.LowDataRateOptimize);

        if (SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor == 6) {
            SX127xLoRa->RegModemConfig1.sx1272bits.ImplicitHeaderModeOn = 1;
            ui->checkBoxImplicit->setChecked(SX127xLoRa->RegModemConfig1.sx1272bits.ImplicitHeaderModeOn);
            radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
        }
    } else {
        SX127xLoRa->RegModemConfig1.sx1276bits.CodingRate = index;
        if (index > 10 && SX127xLoRa->RegModemConfig1.sx1276bits.Bw < 8)
            SX127xLoRa->RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            SX127xLoRa->RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        radio_write(REG_LR_MODEMCONFIG3, SX127xLoRa->RegModemConfig3.octet);
        ui->checkBoxLDRO->setChecked(SX127xLoRa->RegModemConfig3.sx1276bits.LowDataRateOptimize);

        if (SX127xLoRa->RegModemConfig2_timeout.sx1272bits.SpreadingFactor == 6) {
            SX127xLoRa->RegModemConfig1.sx1276bits.ImplicitHeaderModeOn = 1;
            radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
            ui->checkBoxImplicit->setChecked(SX127xLoRa->RegModemConfig1.sx1276bits.ImplicitHeaderModeOn);
        }
    }

}

void FormLora::set_nb_trig_peaks(int n)
{
    SX127xLoRa->RegTest31.bits.detect_trig_same_peaks_nb = n;
    radio_write(REG_LR_TEST31, SX127xLoRa->RegTest31.octet);
}

void FormLora::on_comboBoxCR_currentIndexChanged(int index)
{
    if (!sx1276) {
        SX127xLoRa->RegModemConfig1.sx1272bits.CodingRate = index;
    } else {
        SX127xLoRa->RegModemConfig1.sx1276bits.CodingRate = index;
    }
    radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
}

void FormLora::on_checkBoxCrcOn_clicked(bool checked)
{
    if (sx1276) {
		SX127xLoRa->RegModemConfig2_timeout.sx1276bits.RxPayloadCrcOn = checked;
		radio_write_u16(REG_LR_MODEMCONFIG2, SX127xLoRa->RegModemConfig2_timeout.word);
	} else {
		SX127xLoRa->RegModemConfig1.sx1272bits.RxPayloadCrcOn = checked;
		radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
	fprintf(stderr, "CRCon CodingRate:%d bw:%d\n", SX127xLoRa->RegModemConfig1.sx1276bits.CodingRate,
		SX127xLoRa->RegModemConfig1.sx1276bits.Bw);
	}
}

void FormLora::on_comboBoxBW_currentIndexChanged(int index)
{
	if (!init_done)
		return;

    if (sx1276) {
		SX127xLoRa->RegModemConfig1.sx1276bits.Bw = index;
    } else {
		SX127xLoRa->RegModemConfig1.sx1272bits.Bw = index;
	}

    radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
}

void FormLora::on_spinBoxPreambleLength_editingFinished()
{
	SX127xLoRa->RegPreambleLength = ui->spinBoxPreambleLength->value();
	radio_write_u16(REG_LR_PREAMBLEMSB, SX127xLoRa->RegPreambleLength);
}

bool FormLora::is_implicit_header_mode()
{
    if (sx1276) {
        return SX127xLoRa->RegModemConfig1.sx1276bits.ImplicitHeaderModeOn;
    } else {
        return SX127xLoRa->RegModemConfig1.sx1272bits.ImplicitHeaderModeOn;
    }

}

void FormLora::on_checkBoxImplicit_clicked(bool checked)
{
    if (sx1276) {
		SX127xLoRa->RegModemConfig1.sx1276bits.ImplicitHeaderModeOn = checked;
    } else {
		SX127xLoRa->RegModemConfig1.sx1272bits.ImplicitHeaderModeOn = checked;
	}

    radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
}

void FormLora::set_payload_length(int len)
{
    SX127xLoRa->RegPayloadLength = len;
    ui->spinBoxPayloadLength->setValue(SX127xLoRa->RegPayloadLength);
    radio_write(REG_LR_PAYLOADLENGTH, SX127xLoRa->RegPayloadLength);
}

void FormLora::on_spinBoxSymbTimeout_editingFinished()
{
    SX127xLoRa->RegModemConfig2_timeout.sx1276bits.SymbTimeout = ui->spinBoxSymbTimeout->value();
    radio_write_u16(REG_LR_MODEMCONFIG2, SX127xLoRa->RegModemConfig2_timeout.word);
}

void FormLora::on_checkBoxLDRO_clicked(bool checked)
{
	if (sx1276) {
		SX127xLoRa->RegModemConfig3.sx1276bits.LowDataRateOptimize = checked;
        radio_write(REG_LR_MODEMCONFIG3, SX127xLoRa->RegModemConfig3.octet);
	} else {
		SX127xLoRa->RegModemConfig1.sx1272bits.LowDataRateOptimize = checked;
		radio_write(REG_LR_MODEMCONFIG1, SX127xLoRa->RegModemConfig1.octet);
	}
}

void FormLora::on_checkBoxLoraAGCAutoOn_clicked(bool checked)
{
	if (sx1276) {
		SX127xLoRa->RegModemConfig3.sx1276bits.AgcAutoOn = checked;
        radio_write(REG_LR_MODEMCONFIG3, SX127xLoRa->RegModemConfig3.octet);
	} else {
		SX127xLoRa->RegModemConfig2_timeout.sx1272bits.AgcAutoOn = checked;
		radio_write_u16(REG_LR_MODEMCONFIG2, SX127xLoRa->RegModemConfig2_timeout.word);
	}
}

void FormLora::on_checkBoxTxContinuousMode_clicked(bool checked)
{
	if (sx1276)
		SX127xLoRa->RegModemConfig2_timeout.sx1276bits.TxContinuousMode = checked;
	else
		SX127xLoRa->RegModemConfig2_timeout.sx1272bits.TxContinuousMode = checked;

	radio_write_u16(REG_LR_MODEMCONFIG2, SX127xLoRa->RegModemConfig2_timeout.word);
}

void FormLora::on_listWidgetDIO5_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping2.bits.Dio5Mapping = index.row();
    radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
}

void FormLora::on_listWidgetDIO4_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping2.bits.Dio4Mapping = index.row();
    radio_write(REG_DIOMAPPING2, SX127x.RegDioMapping2.octet);
}

void FormLora::on_listWidgetDIO3_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio3Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void FormLora::on_listWidgetDIO2_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio2Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void FormLora::on_listWidgetDIO1_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio1Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void FormLora::on_listWidgetDIO0_clicked(const QModelIndex &index)
{
    SX127x.RegDioMapping1.bits.Dio0Mapping = index.row();
    radio_write(REG_DIOMAPPING1, SX127x.RegDioMapping1.octet);
}

void FormLora::on_tabWidget_currentChanged(int index)
{
	if (index == 1) {
        diopoll_tid = startTimer(DIO_POLL_RATE);
	} else {
        killTimer(diopoll_tid);
	}
}


void FormLora::on_spinBoxMaxPayloadLength_valueChanged(int arg1)
{
    SX127xLoRa->RegRxMaxPayloadLength = arg1;
    radio_write(REG_LR_RX_MAX_PAYLOADLENGTH, SX127xLoRa->RegRxMaxPayloadLength);
}

void FormLora::on_spinBoxPayloadLength_valueChanged(int arg1)
{
    SX127xLoRa->RegPayloadLength = arg1;
    radio_write(REG_LR_PAYLOADLENGTH, SX127xLoRa->RegPayloadLength);
}
