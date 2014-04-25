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
#ifndef FORMLORA_H
#define FORMLORA_H

#include <QWidget>
#include <QModelIndex>
#include <QTabWidget>

#define DIO_POLL_RATE		50

namespace Ui {
class FormLora;
}

class FormLora : public QWidget
{
    Q_OBJECT
    
public:
    explicit FormLora(QWidget *parent = 0);
    ~FormLora();
    void getRadioStatus_lora(void);
	int _lora_start_tx(void);
	void lora_rx_timer_event(void);
	void init(void);
	bool init_done;
	void rx_pkt(void);
    void timerEvent(QTimerEvent *);
	int diopoll_tid;
	QTabWidget *tabWidget;	// public access
    void set_nb_trig_peaks(int);
    void set_payload_length(int);

private slots:
    void on_comboBoxSF_currentIndexChanged(int index);

    void on_comboBoxCR_currentIndexChanged(int index);

    void on_checkBoxCrcOn_clicked(bool checked);

    void on_comboBoxBW_currentIndexChanged(int index);

    void on_spinBoxPreambleLength_editingFinished();

    void on_checkBoxImplicit_clicked(bool checked);

    //void on_spinBoxPayloadLength_editingFinished();

    void on_spinBoxSymbTimeout_editingFinished();

    void on_checkBoxLDRO_clicked(bool checked);

    void on_checkBoxLoraAGCAutoOn_clicked(bool checked);

    void on_checkBoxTxContinuousMode_clicked(bool checked);

    void on_listWidgetDIO5_clicked(const QModelIndex &index);

    void on_listWidgetDIO4_clicked(const QModelIndex &index);

    void on_listWidgetDIO3_clicked(const QModelIndex &index);

    void on_listWidgetDIO2_clicked(const QModelIndex &index);

    void on_listWidgetDIO1_clicked(const QModelIndex &index);

    void on_listWidgetDIO0_clicked(const QModelIndex &index);

    void on_tabWidget_currentChanged(int index);

    void on_spinBoxMaxPayloadLength_valueChanged(int arg1);

    void on_spinBoxPayloadLength_valueChanged(int arg1);

private:
    Ui::FormLora *ui;
};

#endif // FORMLORA_H
