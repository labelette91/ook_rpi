// ook_rpi.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


/* ===================================================
C code : test.cpp
* ===================================================
*/

#define OTIO_ENABLE
//#define  DOMOTIC
 #define REPORT_SERIAL


#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>    
#include <wiringPi.h>
#include "hager.h"
#include "print.h"

//rfm69 and spi *************************		

#include "RFM69.h"
#include "RFM69registers.h"
#include "SPI.h"


RFM69* radio;

#include "ookdecoder.h"
#include "oregon.h"
#include "domotic.h"

#ifdef REPORT_SERIAL
#include  "reportserialascii.h"
#endif

OregonDecoderV2 orscV2;

#include "HomeEasyTransmitter.h"
HomeEasyTransmitter* easy;

byte data[4];
int 	NbPulse = 0 ;
int     NbPulses =0;


#ifdef OTIO_ENABLE        
#include "decodeotio.h"
DecodeOTIO Otio(3);
#endif

//attention numeo general pin et pas GPIO
int  TXPIN = 5;
int   RXPIN = 5;

int pin = 0;
void PulseLed(int Level)
{
//	return;
	if (Level == 2)
		pin = !pin;
	else
		pin = Level;
	//pulse led1=power
	//pulse led0=verte 
	//echo gpio | sudo tee /sys/class/leds/led1/trigger

	if (pin)
		system("echo 1 | sudo tee /sys/class/leds/led0/brightness > /dev/null");
	else
		system("echo 0 | sudo tee /sys/class/leds/led0/brightness > /dev/null");

}

void reportSerial(const char* s, class DecodeOOK& decoder)
{
#ifdef REPORT_SERIAL
	reportSerialAscii("OSV2", decoder.getData(), decoder.pos);
#endif
#ifdef DOMOTIC
	reportDomotic(decoder.getData(), decoder.pos);
#endif      
}


#define CHANGE 1
#define FALLING 2
#define RISING 3

void  Setup(int rxPin, int txPin , int ledpin)
{
	RXPIN = rxPin;
	TXPIN = txPin;

	//if receiver declared , init receive pin
	if (RXPIN != -1) pinMode(RXPIN, INPUT);

    attachInterrupt(wpiPinToGpio(rxPin) , 0,  CHANGE );

	easy = new HomeEasyTransmitter(TXPIN, 0);
	HagerSetPin                   (TXPIN, 0);

	//init radio module RFF
	radio = new RFM69(0, 0);
	if (radio->initialize(RF69_433MHZ, 1, 100) )
		printf( "HERF: RFM69 initialized TX:%d RX:%d\n", TXPIN, RXPIN);
	else
		printf("HERF: RFM69 not initialized \n", TXPIN, RXPIN);

    //2400 bauds bit rate 3,4
//    radio->writeReg(REG_BITRATEMSB,RF_BITRATEMSB_2400);
//    radio->writeReg(REG_BITRATELSB,RF_BITRATELSB_2400);
    //lna 50 h mieux que 200   
//    radio->writeReg(REG_LNA, RF_LNA_ZIN_50);
//    radio->writeReg(REG_LNA, RF_LNA_ZIN_200);



	if (RXPIN != -1) pinMode(RXPIN, INPUT);

	//si pin TX <> RX
	if (TXPIN != RXPIN)
		radio->setMode(RF69_MODE_SLEEP);
	else
		radio->setMode(RF69_MODE_RX);

//	readListRegs(RegList);
	PrintReg(REG_OPMODE);

	DomoticInit();

}
void Loop(word p)
{
		{
			if (p>0)
            {
				//printf("%d ", p);				if ((p % 16)==0)					printf("\n");
					
				//get pinData
				int pinData = p & 1;

				if (orscV2.nextPulse(p))
				{
					// -1 : on retire le byte de postambule
					if (checksum(orscV2.getData(), orscV2.pos - 1))
					{
						if ((data[3] != orscV2.data[3]) || (data[0] != orscV2.data[0]) || (data[1] != orscV2.data[1]) || (data[2] != orscV2.data[2]))
						{
							PulseLed(2);
							reportSerial("OSV2", orscV2);
							data[0] = orscV2.data[0];
							data[1] = orscV2.data[1];
							data[2] = orscV2.data[2];
							data[3] = orscV2.data[3];
						}
					}
					else
					{
#ifdef REPORT_SERIAL
						Serial.print("\nBad checksum ");
						reportSerial("OSV2", orscV2);
#endif     
					}
					orscV2.resetDecoder();
				}

#ifdef OTIO_ENABLE        
				if (Otio.nextPulse(p, pinData)) {
#ifdef REPORT_SERIAL
					Otio.ReportSerial();
#endif
#ifdef DOMOTIC
					reportDomoticTemp(Otio.getTemperature(), Otio.getId(), 0, Otio.getBatteryLevel());
#endif
					PulseLed(2);
				}
#endif      	
			}
		}

//		readCom();
		DomoticReceive();
		//check domotic send command reception
		//attente une secone max pour emetre si emission en cours -80--> -70
		//pas de reception en cours
		if ((DomoticPacketReceived)
//			&& (radio.canSend(-70))
//			&& (Otio.total_bits == 0)
//			&& (orscV2.total_bits == 0)
//			&& (Rubicson.total_bits == 0)
//		  && (HEasy.total_bits == 0)
//		  && (MD230.total_bits == 0)
			)
		{
			int rssi;

			PulseLed( 1);

			//start receive cmd
			if ((Cmd.ICMND.packettype == 0) && (Cmd.ICMND.cmnd == cmdStartRec)) {
				printf("DomoticStartReceive\n");

				DomoticStartReceive();
			}
			else if ((Cmd.ICMND.packettype == 0) && (Cmd.ICMND.cmnd == cmdSTATUS)) {
				printf("DomoticStatus\n");
				DomoticStatus();
			}
			else if ((Cmd.ICMND.packettype == 0) && (Cmd.ICMND.cmnd == cmdRESET)) {
				printf("cmdRESET\n");
			}
			else
			{

				rssi = radio->readRSSI();

				detachInterrupt(1);
				easy->initPin();
				radio->setMode(RF69_MODE_TX);
				delay(10);

				if (Cmd.LIGHTING2.packettype == pTypeLighting2)
				{  //

					if (Cmd.LIGHTING2.subtype == sTypeHEU) 	         //if home easy protocol : subtype==1
					{
						printf("easy send\n");
						easy->setSwitch(Cmd.LIGHTING2.cmnd, getLightingId(), Cmd.LIGHTING2.unitcode);    // turn on device 0
						Cmd.LIGHTING2.subtype = 1;
					}
					else if (Cmd.LIGHTING2.subtype == sTypeAC) 	         //if hager protocol : subtype==0
					{
						printf("Hager send\n");
						ManageHager(Cmd.LIGHTING2.id4, Cmd.LIGHTING2.unitcode, Cmd.LIGHTING2.cmnd);
						Cmd.LIGHTING2.subtype = 0;
					}
					else
						Cmd.LIGHTING2.subtype = 2;

				}
				else
					Cmd.LIGHTING2.subtype = 3;

				//acknoledge 
				Cmd.LIGHTING2.packettype = pTypeUndecoded;
				Cmd.LIGHTING2.packetlength = 7;
				Cmd.LIGHTING2.id1 = rssi >> 8;
				Cmd.LIGHTING2.id2 = rssi & 0x00ff;
				Cmd.LIGHTING2.id3 = NbPulse >> 8;
				Cmd.LIGHTING2.id4 = NbPulse & 0x00ff;
				Serial.write((byte*)&Cmd.LIGHTING2, Cmd.LIGHTING2.packetlength + 1);

				attachInterrupt(wpiPinToGpio(RXPIN), 0 , CHANGE);
				radio->setMode(RF69_MODE_RX);
				PrintReg(REG_OPMODE);
				pinMode(TXPIN, INPUT);
			}
			PulseLed(0);
			DomoticPacketReceived = false;
		}
}

