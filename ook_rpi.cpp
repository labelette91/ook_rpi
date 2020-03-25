// ook_rpi.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


/* ===================================================
C code : test.cpp
* ===================================================
*/

#define OTIO_ENABLE
#define  DOMOTIC
#define REPORT_SERIAL


#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>    
#include <wiringPi.h>


std::string DeviceR = "/dev/gpiofreq";

#include "ookdecoder.h"
#include "oregon.h"
#include "domotic.h"

#include "print.h"

#ifdef REPORT_SERIAL
#include  "reportserialascii.h"
#endif

int Print::out;
int Print::DomoticOut;

OregonDecoderV2 orscV2;

#include "HomeEasyTransmitter.h"
HomeEasyTransmitter* easy;

// RFM69* radio;



byte data[4];

#define MAXS 4096
int pulse[MAXS];
int 	NbPulse = 0 ;
int     NbPulses =0;

#ifdef OTIO_ENABLE        
#include "decodeotio.h"
DecodeOTIO Otio(3);
#endif

//attention numeo general pin et pas GPIO
#define TXPIN 1 
#define RXPIN 17


int pin = 0;
void PulseLed(int Level)
{
	if (Level == 2)
		pin = !pin;
	else
		pin = Level;
	//pulse led1=power
	//pulse led0=red 
	//echo gpio | sudo tee /sys/class/leds/led1/trigger

	if (pin)
		system("echo 1 | sudo tee /sys/class/leds/led1/brightness > /dev/null");
	else
		system("echo 0 | sudo tee /sys/class/leds/led1/brightness > /dev/null");

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

#define  SLEEP_TIME_IN_US 10000l
int CtMs = 0;

void UpDatePulseCounter(int count )
{
	NbPulses += count;
	CtMs += (SLEEP_TIME_IN_US/1000l);
	if ((CtMs % 1000l) == 0)
	{
		NbPulse = NbPulses;
		NbPulses = 0;
		//			fprintf(stdout,"NbPulse %d\n", NbPulsePerSec);
		//			easy->initPin();
		//			easy->setSwitch(1, 0x55, 1);    // turn on device 0

	}

}
std::string  createVirtualSerial(int &fd);

int ook_rpi_read_drv( int gpio)
{
	FILE* fp;
	std::string Device;
	std::string serial;
	

	//power led sur gpio
	system("echo gpio | sudo tee /sys/class/leds/led1/trigger");

	Device = DeviceR + "17";
	printf("opening %s\n", Device.c_str() );

	fp = fopen(Device.c_str(), "r");
	if (fp == NULL) {
		printf("[ERROR] %s device not found - kernel driver must be started !!\n", Device.c_str());
		exit(1);
	}

	//create virtual tty
	Serial.out = fileno(stdout);
	serial = createVirtualSerial(Serial.DomoticOut);
	if (wiringPiSetup() == -1)
	{
		printf("[ERROR] failed to initialize wiring pi");
		exit(1);
	}
	easy = new HomeEasyTransmitter(TXPIN, 0);

	while (1) {
		int count = fread(pulse, 4, 2048, fp);
		UpDatePulseCounter(count);
		if (count > 0)
		{
			for (int i = 0; i < count; i++)
			{
				word p = pulse[i];
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
						Serial.println("Bad checksum");
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


		//read serial input & fill receive buffe(
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
				DomoticStartReceive();
			}
			else if ((Cmd.ICMND.packettype == 0) && (Cmd.ICMND.cmnd == cmdSTATUS)) {
				DomoticStatus();
			}
			else
			{

/*
fclose(fp);
				fp = fopen(Device.c_str(), "w");
				if (fp == NULL) {printf("[ERROR] %s device not found - kernel driver must be started !!\n", Device.c_str());exit(1);}
*/


//				rssi = radio.readRSSI();

//				detachInterrupt(1);
				easy->initPin();
//				radio.setMode(RF69_MODE_TX);
				delay(10);

				if (Cmd.LIGHTING2.packettype == pTypeLighting2)
				{  //
					printf("easy send\n");
					if (Cmd.LIGHTING2.subtype == sTypeHEU) 	         //if home easy protocol : subtype==1
					{
						easy->setSwitch(Cmd.LIGHTING2.cmnd, getLightingId(), Cmd.LIGHTING2.unitcode);    // turn on device 0
						Cmd.LIGHTING2.subtype = 1;
					}
					else if (Cmd.LIGHTING2.subtype == sTypeAC) 	         //if hager protocol : subtype==0
					{
						//ManageHager(Cmd.LIGHTING2.id4, Cmd.LIGHTING2.unitcode, Cmd.LIGHTING2.cmnd);
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
//				Serial.write((byte*)&Cmd.LIGHTING2, Cmd.LIGHTING2.packetlength + 1);

//				pinMode(PDATA, INPUT);
//				attachInterrupt(1, ext_int_1, CHANGE);
//				radio.setMode(RF69_MODE_RX);
	
			}
			PulseLed(0);
			DomoticPacketReceived = false;
		}
		usleep(SLEEP_TIME_IN_US);
	}
	fclose(fp);
	return 0;
}

int main()
{
	ook_rpi_read_drv(17);
}

#ifdef WIN32
unsigned int millis(void)
{

	long            ms; // Milliseconds
	time_t          s;  // Seconds
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	ms = spec.tv_sec * 1000 + (spec.tv_nsec / 1000000);

	return ms;
}

int usleep(int)
{
	return 0;

}
#else
void scheduler_realtime(void) {

struct sched_param p;
p.__sched_priority = sched_get_priority_max(SCHED_RR);
if (sched_setscheduler(0, SCHED_RR, &p) == -1) {
	printf( "Failed to switch to realtime scheduler.");
}
}

void scheduler_standard(void) {
	struct sched_param p;
	p.__sched_priority = 0;
	if (sched_setscheduler(0, SCHED_OTHER, &p) == -1) {
		printf( "Failed to switch to normal scheduler.");
	}
}

#endif