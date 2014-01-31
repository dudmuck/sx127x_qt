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
#include "spi.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>


tSX127x SX127x;
tSX127xFSK* SX127xFSK = (tSX127xFSK *)&SX127x.ip_regs;
tSX127xLoRa* SX127xLoRa = (tSX127xLoRa *)&SX127x.ip_regs;


flags_t flags;
int rf_buf_len;
uint8_t rf_buf[2048];
unsigned int remaining_;
uint8_t NodeAdrs_to_send;

uint32_t total_tx_pkts;
uint32_t pkt_count;
int pkt_tid;

bool HF;    // for RF switch (false=LF)
bool sx1276;	// false = sx1272

QRadioButton *spi_radioButtonTX;
QRadioButton *spi_radioButtonStandby;
QRadioButton *spi_radioButtonSleep;
QRadioButton *spi_radioButtonFSTX;
QRadioButton *spi_radioButtonFSRX;
QRadioButton *spi_radioButtonRX;
QRadioButton *spi_radioButtonRxSingle;
QRadioButton *spi_radioButtonCAD;

QCheckBox *spi_checkBoxPayloadCrcError;
QCheckBox *spi_checkBoxTxDone;
QCheckBox *spi_checkBoxFHSS;
QCheckBox *spi_checkBoxRxTimeout;
QCheckBox *spi_checkBoxRxDone;
QCheckBox *spi_checkBoxValidHeader;
QCheckBox *spi_checkBoxCadDone;
QCheckBox *spi_checkBoxCadDetected;

uint32_t *xtal_hz;

char tx_busy;

void radio_lora_write_fifo(uint8_t *pbuf, uint8_t len)
{
    uint8_t buf[257];
	int ret;

    // set FIfoAddrPtr to FifoTxBaseAdrs
    radio_write(REG_LR_FIFOADDRPTR, radio_read(REG_LR_FIFOTXBASEADDR));
    // write PayloadLength bytes to the FIFO

    buf[0] = REG_FIFO | 0x80; // bit7 hi for write to radio
    memcpy(buf+1, pbuf, len);

	len++;	// for address-rw byte
    ret = wiringPiSPIDataRW(1, buf, len);
    if (ret != len)
        printf("%d = wiringPiSPIDataRW(,,%d)\n", ret, len);
}

void radio_fsk_write_fifo(uint8_t *pbuf, uint8_t len, bool first_part)
{
    uint8_t buf[257];
    int ret;

    buf[0] = REG_FIFO | 0x80; // bit7 hi for write to radio
    if (first_part) {
        if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) { // start of var-length pkt:
            if (SX127xFSK->RegPktConfig1.bits.AddressFiltering) {
                memcpy(buf+3, pbuf, len);
                buf[1] = rf_buf_len;
                buf[2] = NodeAdrs_to_send;
                len += 2;   // to accomodate these two bytes
            } else {
                memcpy(buf+2, pbuf, len++);  // postinc to accomodate this length byte
                buf[1] = rf_buf_len;
            }
        } else { // start of fixed-length packet:
            if (SX127xFSK->RegPktConfig1.bits.AddressFiltering) {
                memcpy(buf+2, pbuf, len++);  // postinc to accomodate this nodeaddr byte
                buf[1] = NodeAdrs_to_send;
            } else
                memcpy(buf+1, pbuf, len);
        }
    } else { // continuation pkt fragment, no length or address:
        memcpy(buf+1, pbuf, len);
    }


    len++; // accomodation for reg addr+r/nw
    ret = wiringPiSPIDataRW(1, buf, len);
    if (ret != len)
        printf("%d = wiringPiSPIDataRW(,,%d)\n", ret, len);

}

int16_t
radio_read_s16(uint8_t addr)
{
    int16_t ret;
    uint8_t buf[3];
    int l;

    buf[0] = addr;
    l = wiringPiSPIDataRW(1, buf, 3);
    if (l != 3)
        printf("%d = wiringPiSPIDataRW()\n", l);

    ret = buf[1];
    ret <<= 8;
    ret += buf[2];
    return ret;
}

uint8_t radio_read(uint8_t addr)
{
    int ret;
    uint8_t buf[2];

    buf[0] = addr;
    ret = wiringPiSPIDataRW(1, buf, 2);
    if (ret != 2)
        fprintf(stderr, "%d = wiringPiSPIDataRW()\n", ret);


    return buf[1];
}

void radio_write(uint8_t addr, uint8_t val)
{
    int ret;
    uint8_t buf[2];

    buf[0] = addr | 0x80; // bit7 hi for write to radio
    buf[1] = val;
    ret = wiringPiSPIDataRW(1, buf, 2);
    if (ret != 2)
        fprintf(stderr, "%d = wiringPiSPIDataRW()\n", ret);
}

void
show_lora_irqflags()
{
    spi_checkBoxRxTimeout      ->setChecked(SX127xLoRa->RegIrqFlags.bits.RxTimeout);
	spi_checkBoxRxDone         ->setChecked(SX127xLoRa->RegIrqFlags.bits.RxDone);
    spi_checkBoxPayloadCrcError->setChecked(SX127xLoRa->RegIrqFlags.bits.PayloadCrcError);
    spi_checkBoxValidHeader    ->setChecked(SX127xLoRa->RegIrqFlags.bits.ValidHeader);
    spi_checkBoxTxDone         ->setChecked(SX127xLoRa->RegIrqFlags.bits.TxDone);
    spi_checkBoxCadDone        ->setChecked(SX127xLoRa->RegIrqFlags.bits.CadDone);
    spi_checkBoxFHSS           ->setChecked(SX127xLoRa->RegIrqFlags.bits.FhssChangeChannel);
    spi_checkBoxCadDetected    ->setChecked(SX127xLoRa->RegIrqFlags.bits.CadDetected);
}

void showOpMode()
{
    switch (SX127x.RegOpMode.fsk_bits.Mode) {
        case RF_OPMODE_SLEEP:
            spi_radioButtonSleep->setChecked(true);
            break;
        case RF_OPMODE_STANDBY:
            spi_radioButtonStandby->setChecked(true);
            break;
        case RF_OPMODE_SYNTHESIZER_TX:
            spi_radioButtonFSTX->setChecked(true);
            break;
        case RF_OPMODE_TRANSMITTER:
            spi_radioButtonTX->setChecked(true);
            break;
        case RF_OPMODE_SYNTHESIZER_RX:
            spi_radioButtonFSRX->setChecked(true);
            break;
        case RF_OPMODE_RECEIVER:
            spi_radioButtonRX->setChecked(true);
            break;
        case RF_OPMODE_RECEIVER_SINGLE:
            spi_radioButtonRxSingle->setChecked(true);
            break;
        case RF_OPMODE_CAD:
            spi_radioButtonCAD->setChecked(true);
            break;
    }
}

void set_opmode( uint8_t opMode )
{
    if (opMode == RF_OPMODE_TRANSMITTER) {
        if (HF)
            digitalWrite(FEM_CTX_PIN, 1);
        else
            digitalWrite(FEM_CPS_PIN, 0);
    } else if (SX127x.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
        if (HF)
            digitalWrite(FEM_CTX_PIN, 0);
        else
            digitalWrite(FEM_CPS_PIN, 1);
    }

    SX127x.RegOpMode.bits.Mode = opMode;
    radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

	showOpMode();

	if (opMode != RF_OPMODE_TRANSMITTER)
		tx_busy = 0;
}

void reset_flow()
{
    if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
        remaining_ = LENGTH_UNKNOWN;    // prepare for next (manditory for fixed-length)
        rf_buf_len = LENGTH_UNKNOWN;
    } else {
        remaining_ = SX127xFSK->RegPktConfig2.bits.PayloadLength;
        rf_buf_len = SX127xFSK->RegPktConfig2.bits.PayloadLength;
    }
}

// radio_read_fifo(): pass length -1 to read length from radio (start of variable-length packet)
void fsk_radio_read_fifo(uint8_t *pbuf, int arg_len, int max_to_read)
{
    int ret;
    uint8_t buf[FSK_FIFO_SIZE+1];

    if (arg_len == LENGTH_UNKNOWN) {
        uint8_t read_len;
        buf[0] = REG_FIFO;  // bit7 lo for read from radio
        ret = wiringPiSPIDataRW(1, buf, 2);
        if (ret != 2)
            fprintf(stderr, "%d = wiringPiSPIDataRW(,,%d)\n", ret, 2);
        rf_buf_len = buf[1];
        if (rf_buf_len > FSK_FIFO_SIZE) {
            read_len = FSK_FIFO_SIZE - SX127xFSK->RegFifoThreshold.bits.FifoThreshold;
        } else {
            read_len = rf_buf_len;
        }
        max_to_read--;  // because we just read in the length byte
        if (read_len > max_to_read)
            read_len = max_to_read;

        buf[0] = REG_FIFO;  // bit7 lo for read from radio
        if (digitalRead(DIO3_PIN))
            fprintf(stderr, "DIO3\n");
        ret = wiringPiSPIDataRW(1, buf, read_len+1);  // +1 for register address accomodation
        if (ret != (read_len+1))
            fprintf(stderr, "%d = wiringPiSPIDataRW(,,%d)\n", ret, (read_len+1));

        memcpy(pbuf, buf+1, read_len);
        remaining_ = rf_buf_len - read_len;
    }
    else // known pkt length:
    {
        buf[0] = REG_FIFO;  // bit7 lo for read from radio
        if (digitalRead(DIO3_PIN))
            fprintf(stderr, "DIO3\n");
        ret = wiringPiSPIDataRW(1, buf, arg_len+1);  // +1 for register address accomodation
        if (ret != (arg_len+1))
            fprintf(stderr, "%d = wiringPiSPIDataRW(,,%d)\n", ret, (arg_len+1));

        memcpy(pbuf, buf+1, arg_len);
        remaining_ -= arg_len;

    }

}


int fsk_pkt_end__read_fifo()
{
    int i, ret = 0;

    if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
        if (rf_buf_len == LENGTH_UNKNOWN) {
            fsk_radio_read_fifo(rf_buf, LENGTH_UNKNOWN, FSK_FIFO_SIZE);
        } else {
            int read_so_far;
            read_so_far = rf_buf_len - remaining_;
            if (read_so_far < 0) {
                fprintf(stderr, "fail: %d = rf_buf_len - remaining, %d - %d\n", read_so_far, rf_buf_len, remaining_);
                ret = -1;
            } else {
                fsk_radio_read_fifo(&rf_buf[read_so_far], remaining_, FSK_FIFO_SIZE);
            }
        }
    }
    else
    {    // fixed-length packet..
        if (flags.flow_en) {
            unsigned int read_so_far;
            read_so_far = SX127xFSK->RegPktConfig2.bits.PayloadLength - remaining_;
            fsk_radio_read_fifo(&rf_buf[read_so_far], remaining_, remaining_);
            rf_buf_len = read_so_far + remaining_; // should be same as PayloadLength
        } else {
            fsk_radio_read_fifo(rf_buf, SX127xFSK->RegPktConfig2.bits.PayloadLength, SX127xFSK->RegPktConfig2.bits.PayloadLength);
            rf_buf_len = SX127xFSK->RegPktConfig2.bits.PayloadLength;
        }
    }

    SX127xFSK->RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
    if (!SX127xFSK->RegIrqFlags2.bits.FifoEmpty) {
        fprintf(stderr, "irqflags2-end-!FifoEmpty\n");
        i = 0;
        while (!SX127xFSK->RegIrqFlags2.bits.FifoEmpty) {
            fprintf(stderr, "%d fifo stale 0x%02x\n", ++i, radio_read(REG_FIFO));
            SX127xFSK->RegIrqFlags2.octet = radio_read(REG_FSK_IRQFLAGS2);
        }
    }

    return ret;
}

// rising edge of dio0
void dio0_callback(void)
{
    if (!digitalRead(DIO0_PIN))
        fprintf(stderr, "DIO0-level\n");
	if (SX127x.RegOpMode.fsk_bits.LongRangeMode) {
		if (SX127x.RegDioMapping1.bits.Dio0Mapping == 0) {	// RxDone rising
			uint8_t buf[257];
			int ret;
			// check PayloadCrcError flag
			SX127xLoRa->RegIrqFlags.octet = radio_read(REG_LR_IRQFLAGS);
			show_lora_irqflags();
			// clear all set flags on the radio:
			radio_write(REG_LR_IRQFLAGS, SX127xLoRa->RegIrqFlags.octet);
			// if crc ok, read fifo
			if (SX127xLoRa->RegIrqFlags.bits.PayloadCrcError) {
				rf_buf_len = LENGTH_UNKNOWN;
			} else {
				// FifoNbRxBytes indicate count of bytes received thus far
				SX127xLoRa->RegNbRxBytes = radio_read(REG_LR_NBRXBYTES);
				// RegRxDataAddr is dynamic pointer that indicates where lora modem received data has written up to
				// Set FifoPtrAddr to FifoRxCurrentAddr: sets FIFO pointer to location of last packet received
				SX127xLoRa->RegRxDataStartAddr = radio_read(REG_LR_FIFORXCURRENTADDR);
				radio_write(REG_LR_FIFOADDRPTR, SX127xLoRa->RegRxDataStartAddr);
				// that way, RegFifo can be read FifoNbRxBytes times

				buf[0] = REG_FIFO;  // bit7 lo for read from radio
				ret = wiringPiSPIDataRW(1, buf, SX127xLoRa->RegNbRxBytes+1);
				if (ret != SX127xLoRa->RegNbRxBytes+1)
					fprintf(stderr, "%d = wiringPiSPIDataRW(,,%d)\n", ret, SX127xLoRa->RegNbRxBytes+1);
				memcpy(rf_buf, buf+1, SX127xLoRa->RegNbRxBytes);
				rf_buf_len = SX127xLoRa->RegNbRxBytes;
			}
			SX127xLoRa->RxPacketCnt = radio_read_u16(REG_LR_RXPACKETCNTVALUE_MSB);
			SX127xLoRa->RegHopChannel.octet = radio_read(REG_LR_HOPCHANNEL);
			SX127xLoRa->RegPktSnrValue = radio_read(REG_LR_PKTSNRVALUE);
			SX127xLoRa->RegPktRssiValue = radio_read(REG_LR_PKTRSSIVALUE);
			SX127xLoRa->RegModemStatus.octet = radio_read(REG_LR_MODEMSTAT);
			flags.new_rx_pkt = 1;
		} else if (SX127x.RegDioMapping1.bits.Dio0Mapping == 1) {	// TxDone rising
			SX127xLoRa->RegIrqFlags.octet = 0;
			SX127xLoRa->RegIrqFlags.bits.TxDone = 1;
			radio_write(REG_LR_IRQFLAGS, SX127xLoRa->RegIrqFlags.octet);
			// radio standby's itself
			SX127x.RegOpMode.fsk_bits.Mode = RF_OPMODE_STANDBY;
			showOpMode();
		} else {
			fprintf(stderr, "LoRa DIO0 rising Dio0Mapping=%d\n", SX127x.RegDioMapping1.bits.Dio0Mapping);
		}
	} else {
		if (SX127x.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
			set_opmode(RF_OPMODE_STANDBY);
			if (remaining_ > 0) {
				fprintf(stderr, "PacketSent: R=%d\n", remaining_);
			}
			tx_busy = 0;
		} else if (SX127x.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
			if (SX127xFSK->RegPktConfig2.bits.DataModePacket) {
				while (flags.busy_fifopull) {
					fprintf(stderr, "0B");
					usleep(10000);
				}
				flags.busy_fifopull = 1;
				if (SX127xFSK->RegRxConfig.bits.AfcAutoOn) {
					SX127xFSK->RegAfc = radio_read_s16(REG_FSK_AFCMSB);
					flags.new_afc_value = 1;
				}
				if (fsk_pkt_end__read_fifo() == 0)
					flags.new_rx_pkt = 1;

				flags.busy_fifopull = 0;
			} // else continuous todo
			else {
				fprintf(stderr, "dio0 rx continuous\n");
			}
		} else
			fprintf(stderr, "dio0, mode:%d\n", SX127x.RegOpMode.bits.Mode);
	}

}

/*void continue_pn9_tx_fifo()
{
    unsigned int stop = FSK_FIFO_SIZE - SX127xFSK->RegFifoThreshold.bits.FifoThreshold;
    for (rf_buf_len = 0; rf_buf_len < stop; rf_buf_len++)
        rf_buf[rf_buf_len] = get_pn9_byte();
}*/

// edge of dio1
void dio1_callback(void)
{
    if (SX127x.RegOpMode.bits.LongRangeMode) {
		fprintf(stderr, "dio1_callback lora\n");
	} else {
		if (flags.flow_en && SX127x.RegDioMapping1.bits.Dio1Mapping == 0) { // if Dio1 is FifoLevel
			if (SX127x.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
				unsigned int fifo_space = FSK_FIFO_SIZE - SX127xFSK->RegFifoThreshold.bits.FifoThreshold;
				unsigned int sent_so_far = rf_buf_len - remaining_;
				if (digitalRead(DIO3_PIN))
					fprintf(stderr, "DIO3\n");
				if (remaining_ < fifo_space) { // last fill:
				   radio_fsk_write_fifo(&rf_buf[sent_so_far], remaining_, false);
				   remaining_ = 0;
				   flags.flow_en = 0;
				} else { // can only fill to end of fifo
				   radio_fsk_write_fifo(&rf_buf[sent_so_far], fifo_space, false);
				   remaining_ -= fifo_space;
				}
			}
			else if (SX127x.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER)
			{
				if (flags.busy_fifopull) {
					return;
				}
				flags.busy_fifopull = 1;
				if (SX127xFSK->RegPktConfig1.bits.PacketFormatVariable) {
					if (rf_buf_len == LENGTH_UNKNOWN) {
						fsk_radio_read_fifo(rf_buf, LENGTH_UNKNOWN, SX127xFSK->RegFifoThreshold.bits.FifoThreshold);
					} else {
						int read_so_far;
						read_so_far = rf_buf_len - remaining_;
						if (remaining_ > SX127xFSK->RegFifoThreshold.bits.FifoThreshold) {
							fsk_radio_read_fifo(&rf_buf[read_so_far], SX127xFSK->RegFifoThreshold.bits.FifoThreshold, SX127xFSK->RegFifoThreshold.bits.FifoThreshold);
						}
					}
				} else { // fixed-length-pkt fifo read
					if (remaining_ > SX127xFSK->RegFifoThreshold.bits.FifoThreshold) {
						int read_so_far = SX127xFSK->RegPktConfig2.bits.PayloadLength - remaining_;
						if (read_so_far >= 0)
							fsk_radio_read_fifo(&rf_buf[read_so_far], SX127xFSK->RegFifoThreshold.bits.FifoThreshold, SX127xFSK->RegFifoThreshold.bits.FifoThreshold);
						else
							fprintf(stderr, "read_so_far:%d R=%d\n", read_so_far, remaining_);
					}
				}
				flags.busy_fifopull = 0;
				SX127xFSK->RegRssiValue = radio_read(REG_FSK_RSSIVALUE);
			}
		} // ..if fifolevel continue

	}

} // ...dio1_callback()

edge_e dio1_edge;

void
set_dio1_edge(edge_e arg_edge)
{
    const char *modeS;
    char str[64];

    if (arg_edge == FALLING_EDGE)
        modeS = "falling";
    else if (arg_edge == RISING_EDGE)
        modeS = "rising";
    else
        modeS = "both";

    sprintf(str, "gpio edge %d %s", DIO1_PIN, modeS);
    if (system(str)) {
        perror(str);
        return;
    }

    dio1_edge = arg_edge;
}

int Spi::open()
{
#ifdef SYS_MODE
    if (wiringPiSetupSys() < 0) {   /* doesnt need root */
#else
    if (wiringPiSetup(void Spi::dio0_callback(void)) < 0) {  /* needs sudo */
#endif
        fprintf(stderr, "wiringPiSetup()\n");
        return -1;
    }

    spi_fd = wiringPiSPISetup(1, 500000);
	/* wiringPi failure calls exit() */
    if (spi_fd < 0) {
        perror("wiringPiSPISetup");
        return -1;
    }

    if (wiringPiISR (DIO0_PIN, INT_EDGE_RISING, &dio0_callback) < 0)
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
        return -1;
    }

    dio1_edge = FALLING_EDGE;
    if (wiringPiISR (DIO1_PIN, INT_EDGE_FALLING, &dio1_callback) < 0)
    {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
        return -1;
    }


#ifdef SYS_MODE
    {
	//DIO0_PIN        17  // pin11
	//DIO1_PIN        27  // pin13
	//DIO2_PIN        22  // pin15
	//DIO3_PIN        23  // pin16
	//DIO4_PIN        24  // pin18
	//DIO5_PIN        25  // pin22
	//HWRESET_PIN     4   // pin7
        char str[64];
        sprintf(str, "gpio export %d out", FEM_CPS_PIN);  // sx1276 LF rf switch
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d out", FEM_CTX_PIN);  // sx1272 HF rf switch
        if (system(str)) {
            perror(str);
			return -1;
		}

        sprintf(str, "gpio export %d in", DIO0_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d in", DIO1_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d in", DIO2_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d in", DIO3_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d in", DIO4_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
        sprintf(str, "gpio export %d in", DIO5_PIN);  //
        if (system(str)) {
            perror(str);
			return -1;
		}
    }
#endif /* SYS_MODE */
	return 0;
}

Spi::Spi()
{
}

void radio_write_u16(uint8_t addr, uint16_t val)
{
    int ret;
    uint8_t buf[3];

    buf[0] = addr | 0x80;
    buf[1] = val >> 8;
    buf[2] = val & 0xff;

    ret = wiringPiSPIDataRW(1, buf, 3);
    if (ret != 3)
        fprintf(stderr, "%d = wiringPiSPIDataRW()\n", ret);
}

uint16_t radio_read_u16(uint8_t addr)
{
    uint16_t ret;
    uint8_t buf[3];
    int l;

    buf[0] = addr;
    l = wiringPiSPIDataRW(1, buf, 3);
    if (l != 3)
        fprintf(stderr, "%d = wiringPiSPIDataRW()\n", l);

    ret = buf[1];
    ret <<= 8;
    ret += buf[2];
    return ret;
}

void setMHz(float MHz)
{
    uint8_t buf[4];

    if (MHz <= 525)
        HF = false;
    else
        HF = true;

    int frf = MHz / FRF_MHZ_C;
    buf[0] = REG_FRFMSB | 0x80;
    buf[1] = frf >> 16;
    buf[2] = (frf >> 8) & 0xff;
    buf[3] = frf & 0xff;
    wiringPiSPIDataRW(1, buf, 4);
}

float Spi::getMHz()
{
    int ret;
    uint8_t buf[4];
    int frf;
    float MHz;

    buf[0] = REG_FRFMSB;
    ret = wiringPiSPIDataRW(1, buf, 4);
    if (ret != 4) {
        fprintf(stderr, "getMHz: %d = wiringPiSPIDataRW()\n", ret);
        return 0;
    }
    frf = (buf[1] << 16) + (buf[2] << 8) + buf[3];

    MHz = frf * FRF_MHZ_C;
    if (MHz <= 525)
        HF = false;
    else
        HF = true;

    return MHz;
}

void Spi::hw_reset()
{
    char str[64];
    sprintf(str, "gpio export %d out", HWRESET_PIN);
    if (system(str))
        perror(str);

    digitalWrite(HWRESET_PIN, 1);

    usleep(100000);
    digitalWrite(HWRESET_PIN, 0);

    sprintf(str, "gpio export %d in", HWRESET_PIN);
    if (system(str))
        perror(str);
}

void set_bitrate(unsigned int bps)
{
    uint16_t tmpBitrate = ( uint16_t )( ( double )*xtal_hz / ( double )bps );
    float actual = *xtal_hz / (float)tmpBitrate;
    float diff = actual - bps;
    if (diff < 1)
        radio_write_u16(REG_FSK_BITRATEMSB, tmpBitrate);
    else {
        uint8_t frac = round( ((*xtal_hz-(bps*tmpBitrate)) / (float)bps) * 16 );
		if (sx1276)
			radio_write(REG_BITRATEFRAC_SX1276, frac);
		else
			radio_write(REG_BITRATEFRAC_SX1272, frac);

        radio_write_u16(REG_FSK_BITRATEMSB, tmpBitrate);
    }
}

void set_fdev(unsigned int hz)
{
    uint16_t tmpFdev = ( uint16_t )( ( double )hz / ( double )FRF_HZ_C);
    radio_write_u16(REG_FSK_FDEVMSB, tmpFdev);
}

uint32_t ComputeRxBw( uint8_t mantisse, uint8_t exponent ) {
    // rxBw -- only valid in FSK mode
    if (SX127x.RegOpMode.fsk_bits.ModulationType == 0)
        return *xtal_hz / (mantisse * (1 << (exponent+2)));
    else
        return *xtal_hz / (mantisse * (1 << (exponent+3)));
}

void ComputeRxBwMantExp( uint32_t rxBwValue, uint8_t* mantisse, uint8_t* exponent )
{
    uint8_t tmpExp, tmpMant;
    double tmpRxBw;
    double rxBwMin = 10e6;

    for( tmpExp = 0; tmpExp < 8; tmpExp++ ) {
        for( tmpMant = 16; tmpMant <= 24; tmpMant += 4 ) {
            tmpRxBw = ComputeRxBw(tmpMant, tmpExp);
            if( fabs( tmpRxBw - rxBwValue ) < rxBwMin ) {
                rxBwMin = fabs( tmpRxBw - rxBwValue );
                *mantisse = tmpMant;
                *exponent = tmpExp;
            }
        }
    }
}

void set_rx_dcc_bw_hz(uint32_t bw_hz, bool afc)
{
    uint8_t mantisse = 0;
    uint8_t exponent = 0;

    ComputeRxBwMantExp( bw_hz, &mantisse, &exponent );
    switch( mantisse ) {
        case 16:
            SX127xFSK->RegRxBw.bits.Mantissa = 0;
            break;
        case 20:
            SX127xFSK->RegRxBw.bits.Mantissa = 1;
            break;
        case 24:
            SX127xFSK->RegRxBw.bits.Mantissa = 2;
            break;
        default:
            // Something went terribely wrong
            fprintf(stderr, "bad maintisse:%d, bw_hz=%d\n", mantisse, bw_hz);
            break;
    }
    SX127xFSK->RegRxBw.bits.Exponent = exponent;

    if (afc)
        radio_write(REG_FSK_AFCBW, SX127xFSK->RegRxBw.octet);
    else
        radio_write(REG_FSK_RXBW, SX127xFSK->RegRxBw.octet);
}

uint32_t get_rx_bw_hz(uint8_t addr)
{
    RegRxBw_t reg_bw;
    uint8_t mantissa;

    reg_bw.octet = radio_read(addr);
    switch (reg_bw.bits.Mantissa) {
        case 0: mantissa = 16; break;
        case 1: mantissa = 20; break;
        case 2: mantissa = 24; break;
        default: mantissa = 0; break;
    }

    if (addr == REG_FSK_RXBW)
        SX127xFSK->RegRxBw.octet = reg_bw.octet;
    else if (addr == REG_FSK_AFCBW)
        SX127xFSK->RegAfcBw.octet = reg_bw.octet;

    return ComputeRxBw(mantissa, reg_bw.bits.Exponent);
}

int lfsr;

uint8_t get_pn9_byte()
{
    int xor_out;
    uint8_t bp, ret = 0;

    for (bp = 0x80; bp > 0; bp >>= 1) {
        xor_out = ((lfsr >> 5) & 1) ^ (lfsr & 1);
        lfsr = (lfsr >> 1) | (xor_out << 8);
        if (lfsr & 0x100)
            ret |= bp;
    }

    /*xor_out = ((lfsr >> 5) & 0xf) ^ (lfsr & 0xf);	// four bits at a time (four clocks)
    lfsr = (lfsr >> 4) | (xor_out << 5);		// four bits at a time (four clocks)
    ret |= lfsr >> 1;*/

    return ret;
}

void
read_all_pins(uint8_t *pins)
{
	if (digitalRead(DIO0_PIN))
		*pins |= 0x01;
	else
		*pins &= ~0x01;

	if (digitalRead(DIO1_PIN))
		*pins |= 0x02;
	else
		*pins &= ~0x02;

	if (digitalRead(DIO2_PIN))
		*pins |= 0x04;
	else
		*pins &= ~0x04;

	if (digitalRead(DIO3_PIN))
		*pins |= 0x08;
	else
		*pins &= ~0x08;

	if (digitalRead(DIO4_PIN))
		*pins |= 0x10;
	else
		*pins &= ~0x10;

	if (digitalRead(DIO5_PIN))
		*pins |= 0x20;
	else
		*pins &= ~0x20;
}

