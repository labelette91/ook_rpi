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
#include "hager.h"
#include "print.h"

//rfm69 and spi *************************		

#include "RFM69.h"
#include "RFM69registers.h"
#include "SPI.h"

SPIClass SPI;

RFM69* radio;


int SPI_CHAN = 1;
int Spi_speed = 500000;

std::string DeviceR = "/dev/gpiofreq";

#include "ookdecoder.h"
#include "oregon.h"
#include "domotic.h"


#ifdef REPORT_SERIAL
#include  "reportserialascii.h"
#endif

int Print::out;
int Print::DomoticOut;

OregonDecoderV2 orscV2;

#include "HomeEasyTransmitter.h"
HomeEasyTransmitter* easy;

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
#define TXPIN 24 
#define RXPIN 24

#include "rfmPrint.cpp"

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
		static int lastrssi=0;

		/*
		
		int rssi = radio->readRSSI();
		if (lastrssi != rssi) 
		{
			printf("rssi:%d ", rssi);
			fprintf(stdout, " NbPulse %d\n", NbPulse);
		}
		lastrssi = rssi;
		*/
	}

}
std::string  createVirtualSerial(int &fd);

int ook_rpi_read_drv( char * rxgpio, int debug)
{
	FILE* fp;
	std::string Device;
	std::string serial;
	

	//power led sur gpio
	system("echo gpio | sudo tee /sys/class/leds/led1/trigger");

	if (wiringPiSetup() == -1)	{printf("[ERROR] failed to initialize wiring pi");exit(1);	}

	//if receiver declared , init receive pin
	if (RXPIN != -1) pinMode(RXPIN, INPUT);

	Device = DeviceR + rxgpio ;
	printf("opening %s\n", Device.c_str() );

	//open pulse driver
	fp = fopen(Device.c_str(), "r");
	if (fp == NULL) {printf("[ERROR] %s device not found - kernel driver must be started !!\n", Device.c_str());		/*exit(1);*/ 	}

	//create virtual tty
	Serial.out = fileno(stdout);
	serial = createVirtualSerial(Serial.DomoticOut);
	easy = new HomeEasyTransmitter(TXPIN, 0);
	HagerSetPin                   (TXPIN, 0);

	//init SPI
	if (SPI.Setup(SPI_CHAN, Spi_speed))
		printf( "failed to open the SPI bus: ");

	//init radio module RFF
	radio = new RFM69(0, 0);
	if (radio->initialize(RF69_433MHZ, 1, 100) )
		printf( "HERF: RFM69 initialized TX:%d RX:%d\n", TXPIN, RXPIN);
	else
		printf("HERF: RFM69 not initialized \n", TXPIN, RXPIN);

	if (RXPIN != -1) pinMode(RXPIN, INPUT);

	//si pin TX <> RX
	if (TXPIN != RXPIN)
		radio->setMode(RF69_MODE_SLEEP);
	else
		radio->setMode(RF69_MODE_RX);

	readListRegs(RegList);
	PrintReg(REG_OPMODE);

	printf("running\n" );
	while (1) {
		int count = 0;
	 count = fread(pulse, 4, 2048, fp);
		UpDatePulseCounter(count);
		if (count > 0)
		{
			for (int i = 0; i < count; i++)
			{
				word p = pulse[i];
				//printf("%d ", p);
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
		DomoticPacketReceived = 0;
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

				fclose(fp);
//				fp = fopen(Device.c_str(), "w");
//				if (fp == NULL) {printf("[ERROR] %s device not found - kernel driver must be started !!\n", Device.c_str());exit(1);}



				rssi = radio->readRSSI();

//				detachInterrupt(1);
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
				Serial.Write((byte*)&Cmd.LIGHTING2, Cmd.LIGHTING2.packetlength + 1);

//				pinMode(PDATA, INPUT);
//				attachInterrupt(1, ext_int_1, CHANGE);
				radio->setMode(RF69_MODE_RX);

				//open pulse driver
				fp = fopen(Device.c_str(), "r");
				if (fp == NULL) {
					printf("[ERROR] %s device not found - kernel driver must be started !!\n", Device.c_str());
					exit(1);
				}


			}
			PulseLed(0);
			DomoticPacketReceived = false;
		}
		usleep(SLEEP_TIME_IN_US);
	}
	fclose(fp);
	return 0;
}

int main(int argc , char** argv)
{
	int debug = 0xff;
	char* rxPin =(char*) "17";
	if (argc > 1)
		rxPin = argv[1];
	if (argc > 2)
		debug = atoi(argv[2]);

	printf("%s %d\n", rxPin,debug);
	ook_rpi_read_drv(rxPin, debug);
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