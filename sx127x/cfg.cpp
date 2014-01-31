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
#include "cfg.h"
#include "spi.h"
#include <string.h>

const char *common_regnames_start_lora[] = {
    "RegFifo",
    "RegOpMode",
    "RegRes02",
    "RegRes03",
    "RegRes04",
    "RegRes05",
    "RegFrfMsb",
    "RegFrfMid",
    "RegFrfLsb",
    "RegPaConfig",
    "RegPaRamp",
    "RegOcp",
    "RegLna"
};

const char *common_regnames_start_fsk[] = {
    "RegFifo",
    "RegOpMode",
    "RegBitrateMsb",
    "RegBitrateLsb",
    "RegFdevMsb",
    "RegFdevLsb",
    "RegFrfMsb",
    "RegFrfMid",
    "RegFrfLsb",
    "RegPaConfig",
    "RegPaRamp",
    "RegOcp",
    "RegLna"
};

const char *fsk_regnames[] = {
    "RegRxConfig",	//	0x0D
    "RegRssiConfig",	//	0x0E
    "RegRssiCollision",	//	0x0F
    "RegRssiThresh",	//	0x10
    "RegRssiValue",	//	0x11
    "RegRxBw",	//	0x12
    "RegAfcBw",	//	0x13
    "RegOokPeak",	//	0x14
    "RegOokFix",	//	0x15
    "RegOokAvg",	//	0x16
    "RegRes17",	//	0x17
    "RegRes18",	//	0x18
    "RegRes19",	//	0x19
	"RegAfcFei",	//	0x1A
	"RegAfcMsb",	//	0x1B
	"RegAfcLsb",	//	0x1C
	"RegFeiMsb",	//	0x1D
	"RegFeiLsb",	//	0x1E
	"RegPreambleDetect",	//	0x1F
	"RegRxTimeout1",	//	0x20
	"RegRxTimeout2",	//	0x21
	"RegRxTimeout3",	//	0x22
	"RegRxDelay",	//	0x23
	"RegOsc",	//	0x24
	"RegPreambleMsb",	//	0x25
	"RegPreambleLsb",	//	0x26
	"RegSyncConfig",	//	0x27
	"RegSyncValue1",	//	0x28
	"RegSyncValue2",	//	0x29
	"RegSyncValue3",	//	0x2A
	"RegSyncValue4",	//	0x2B
	"RegSyncValue5",	//	0x2C
	"RegSyncValue6",	//	0x2D
	"RegSyncValue7",	//	0x2E
	"RegSyncValue8",	//	0x2F
	"RegPacketConfig1",	//	0x30
	"RegPacketConfig2",	//	0x31
	"RegPayloadLength",	//	0x32
	"RegNodeAdrs",	//	0x33
	"RegBroadcastAdrs",	//	0x34
	"RegFifoThresh",	//	0x35
	"RegSeqConfig1",	//	0x36
	"RegSeqConfig1",	//	0x37
	"RegTimerResol",	//	0x38
	"RegTimer1Coef",	//	0x39
	"RegTimer2Coef",	//	0x3A
	"RegImageCal",	//	0x3B
	"RegTemp",	//	0x3C
	"RegLowBat",	//	0x3D
	"RegIrqFlags1",	//	0x3E
	"RegIrqFlags2",	//	0x3F
};

const char *lora_regnames[] = {
	"RegFifoAddrPtr",	//	0x0D
	"RegFifoTxBaseAddr",	//	0x0E
	"RegFifoRxBaseAddr",	//	0x0F
	"RegFifoRxCurrentAddr",	//	0x10
	"RegIrqFlagsMask",	//	0x11
	"RegIrqFlags",	//	0x12
	"RegRxNbBytes",	//	0x13
	"RegRxHeaderCntValueMsb",	//	0x14
	"RegRxHeaderCntValueLsb",	//	0x15
	"RegRxPacketCntValueMsb",	//	0x16
	"RegRxPacketCntValueLsb",	//	0x17
	"RegModemStat",	//	0x18
	"RegPktSnrValue",	//	0x19
	"RegPktRssiValue",	//	0x1A
	"RegRssiValue",	//	0x1B
	"RegHopChannel",	//	0x1C
	"RegModemConfig1",	//	0x1D
	"RegModemConfig2",	//	0x1E
	"RegSymbTimeoutLsb",	//	0x1F
	"RegPreambleMsb",	//	0x20
	"RegPreambleLsb",	//	0x21
	"RegPayloadLength",	//	0x22
	"RegMaxPayloadLength",	//	0x23
	"RegHopPeriod",	//	0x24
	"RegFifoRxByteAddr",	//	0x25
	"RegModemConfig3",	//	0x26
	"RegTest27",	//	0x27
	"RegTest28",	//	0x28
	"RegTest29",	//	0x29
	"RegTest2A",	//	0x2A
	"RegTest2B",	//	0x2B
	"RegTest2C",	//	0x2C
	"RegTest2D",	//	0x2D
	"RegTest2E",	//	0x2E
	"RegTest2F",	//	0x2F
	"RegTest30",	//	0x30
	"RegTest31",	//	0x31
	"RegTest32",	//	0x32
	"RegTest33",	//	0x33
	"RegTest34",	//	0x34
	"RegTest35",	//	0x35
	"RegTest36",	//	0x36
	"RegTest37",	//	0x37
	"RegTest38",	//	0x38
	"RegTest39",	//	0x39
	"RegTest3A",	//	0x3A
	"RegTest3B",	//	0x3B
	"RegTest3C",	//	0x3C
	"RegTest3D",	//	0x3D
	"RegTest3E",	//	0x3E
	"RegTest3F",	//	0x3F
};

const char *common_regnames_end_1276[] = {
	"RegDioMapping1",	// 0x40
	"RegDioMapping2",	// 0x41
	"RegVersion",	// 0x42
	"RegTest43",	// 0x43
	"RegPllHop", // 0x44
	"RegTest45",	// 0x45
	"RegTest46",	// 0x46
	"RegTest47",	// 0x47
	"RegTest48",	// 0x48
	"RegTest49",	// 0x49
	"RegTest4A",	// 0x4a
	"RegTcxo",	// 0x4b
	"RegTest4C",	// 0x4c
	"RegPaDac",	// 0x4d
	"RegTest4E",	// 0x4e
	"RegTest4F",	// 0x4f
	"RegTest50",	// 0x50
	"RegTest51",	// 0x51
	"RegTest52",	// 0x52
	"RegTest53",	// 0x53
	"RegTest54",	// 0x54
	"RegTest55",	// 0x55
	"RegTest56",	// 0x56
	"RegTest57",	// 0x57
	"RegTest58",	// 0x58
	"RegTest59",	// 0x59
	"RegTest5A",	// 0x5a
	"RegFormerTemp",	// 0x5b
	"RegTest5C",	// 0x5c
	"RegTest5D",	// 0x5d
	"RegTest5E",	// 0x5e
	"RegTest5F",	// 0x5f
	"RegTest60",	// 0x60
	"RegAgcRef",	// 0x61
	"RegAgcThresh1",	// 0x62
	"RegAgcThresh2",	// 0x63
	"RegAgcThresh3",	// 0x64
	"RegTest65",	// 0x65
	"RegTest66",	// 0x66
	"RegTest67",	// 0x67
	"RegTest68",	// 0x68
	"RegTest69",	// 0x69
	"RegTest6A",	// 0x6a
	"RegTest6B",	// 0x6b
    "RegTest6C",	// 0x6c
    "RegTest6D",	// 0x6d
    "RegTest6E",	// 0x6e
    "RegTest6F",	// 0x6f
    "RegPll",	// 0x70
    "RegTest71",	// 0x71
    "RegTest72",	// 0x72
    "RegTest73",	// 0x73
    "RegTest74",	// 0x74
    "RegTest75",	// 0x75
    "RegTest76",	// 0x76
    "RegTest77",	// 0x77
    "RegTest78",	// 0x78
    "RegTest79",	// 0x79
    "RegTest7A",	// 0x7a
    "RegTest7B",	// 0x7b
    "RegTest7C",	// 0x7c
    "RegTest7D",	// 0x7d
    "RegTest7E",	// 0x7e
    "RegTest7F"	// 0x7f
};	// ...sx1276

const char *common_regnames_end_1272[] = {
	"RegDioMapping1",	// 0x40
	"RegDioMapping2",	// 0x41
	"RegVersion",	// 0x42
	"RegAgcRef",	// 0x43
	"RegAgcThresh1",	// 0x44
	"RegAgcThresh2",	// 0x45
	"RegAgcThresh3",	// 0x46
	"RegTest47",	// 0x47
	"RegTest48",	// 0x48
	"RegTest49",	// 0x49
	"RegTest4A",	// 0x4a
	"RegPllHop",	// 0x4b
	"RegTest4C",	// 0x4c
	"RegTest4D",	// 0x4d
	"RegTest4E",	// 0x4e
	"RegTest4F",	// 0x4f
	"RegTest50",	// 0x50
	"RegTest51",	// 0x51
	"RegTest52",	// 0x52
	"RegTest53",	// 0x53
	"RegTest54",	// 0x54
	"RegTest55",	// 0x55
	"RegTest56",	// 0x56
	"RegTest57",	// 0x57
	"RegTcxo",	// 0x58
	"RegTest59",	// 0x59
	"RegPaDac",	// 0x5a
	"RegTest5B",	// 0x5b
	"RegPll",	// 0x5c
	"RegTest5D",	// 0x5d
	"RegPllLowPn",	// 0x5e
	"RegTest5F",	// 0x5f
	"RegTest60",	// 0x60
	"RegTest61",	// 0x61
	"RegTest62",	// 0x62
	"RegTest63",	// 0x63
	"RegTest64",	// 0x64
	"RegTest65",	// 0x65
	"RegTest66",	// 0x66
	"RegTest67",	// 0x67
	"RegTest68",	// 0x68
	"RegTest69",	// 0x69
	"RegTest6A",	// 0x6a
	"RegTest6B",	// 0x6b
    "RegFormerTemp",	// 0x6c
    "RegTest6D",	// 0x6d
    "RegTest6E",	// 0x6e
    "RegTest6F",	// 0x6f
    "RegBitrateFrac",	// 0x70
    "RegTest71",	// 0x71
    "RegTest72",	// 0x72
    "RegTest73",	// 0x73
    "RegTest74",	// 0x74
    "RegTest75",	// 0x75
    "RegTest76",	// 0x76
    "RegTest77",	// 0x77
    "RegTest78",	// 0x78
    "RegTest79",	// 0x79
    "RegTest7A",	// 0x7a
    "RegTest7B",	// 0x7b
    "RegTest7C",	// 0x7c
    "RegTest7D",	// 0x7d
    "RegTest7E",	// 0x7e
    "RegTest7F"	// 0x7f
};


Cfg::Cfg()
{
	xtal_hz = XTAL_FREQ_DEFAULT;
}

void Cfg::save(char* fname)
{
    int i, v;
	const char **ip;
	const char **common_regnames_end;
	const char **common_regnames_start;
    RegOpMode_t test_regopmode;

    if (fname != NULL) {
        strncpy(path, fname, sizeof(path));
    }

    f = fopen(path, "w");
    if (f == NULL) {
        perror(path);
        return;
    }

    fprintf(f, "#Type   Register Name   Address[Hex]    Value[Hex]\r\n");

	test_regopmode.octet = radio_read(REG_OPMODE);
	if (test_regopmode.bits.LongRangeMode)
		common_regnames_start = common_regnames_start_lora;
	else
		common_regnames_start = common_regnames_start_fsk;

    for (i = 0; i < 0x0d; i++) {
        v = radio_read(i);
		if (i == REG_OPMODE)
            SX127x.RegOpMode.octet = v;
        fprintf(f, "REG\t%s\t0x%02x\t0x%02x\r\n", common_regnames_start[i], i, v);
    }

    if (SX127x.RegOpMode.bits.LongRangeMode)
		ip = &lora_regnames[0];
	else
		ip = &fsk_regnames[0];

    for (i = 0x0d; i < 0x40; i++) {
        v = radio_read(i);
        fprintf(f, "REG\t%s\t0x%02x\t0x%02x\r\n", ip[i-0x0d], i, v);
	}

	if (sx1276)
		common_regnames_end = common_regnames_end_1276;
	else
		common_regnames_end = common_regnames_end_1272;


    for (i = 0x40; i < 0x80; i++) {
        v = radio_read(i);
        fprintf(f, "REG\t%s\t0x%02x\t0x%02x\r\n", common_regnames_end[i-0x40], i, v);
    }

    fclose(f);
}

void Cfg::parseFile()
{
    char line[128];
    char* ret;
    RegOpMode_t test_regopmode;

    SX127x.RegOpMode.bits.Mode = RF_OPMODE_SLEEP;
    radio_write(REG_OPMODE, SX127x.RegOpMode.octet);

	do {
		test_regopmode.octet = radio_read(REG_OPMODE);
	} while (test_regopmode.bits.Mode != RF_OPMODE_SLEEP);
	
    while ((ret = fgets(line, sizeof(line), f)) != NULL) {
        char* tok = strtok(line, "\t");
        if (!strcmp(tok, "REG")) {
            char *addr, *value;
            tok = strtok(NULL, "\t");   // reg name
            addr = strtok(NULL, "\t");
            value = strtok(NULL, "\t");
            if (addr != REG_FIFO) {
                int a, d, r;
                sscanf(addr, "%x", &a);
                sscanf(value, "%x", &d);
				if (a == REG_OPMODE) {
					RegOpMode_t new_regopmode;
					new_regopmode.octet = d;
					if (new_regopmode.bits.LongRangeMode != test_regopmode.bits.LongRangeMode) {
						new_regopmode.bits.Mode = RF_OPMODE_SLEEP;
						radio_write(REG_OPMODE, new_regopmode.octet);
						do {
							new_regopmode.octet = radio_read(REG_OPMODE);
						} while (new_regopmode.bits.Mode != RF_OPMODE_SLEEP);
					}
				}
                radio_write(a, d);
                r = radio_read(a);
                if (d != r)
                    fprintf(stderr, "at %02x: wrote %02x, read back %02x\n", a, d, r);
            }
        } else if (!strcmp(tok, "PKT")) {
            tok = strtok(NULL, "\t");   // TODO
        } else if (!strcmp(tok, "XTAL")) {
            tok = strtok(NULL, "\t");
			sscanf(tok, "%u", &xtal_hz);
        } else if (tok[0] == '#') {
            // ignore comment line
        } else {
            fprintf(stderr, "tok:\"%s\"\n", tok);
            tok = strtok(NULL, "\t");
            fprintf(stderr, "2nd call:\"%s\"\n", tok);
            tok = strtok(NULL, "\t");
            fprintf(stderr, "3rd call:\"%s\"\n", tok);
        }
    }
}

void Cfg::openFile(char* str)
{
    if (strlen(str) == 0)
        return; // no file selected

    f = fopen(str, "r");
    if (f == NULL) {
        perror(str);
        return;
    }

    strncpy(path, str, sizeof(path));

    parseFile();
    fclose(f);
}

