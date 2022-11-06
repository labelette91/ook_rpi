// ook_rpi.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


/* ===================================================
C code : test.cpp
* ===================================================
*/



#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>    
#include <wiringPi.h>
#include "print.h"
#include  "reportSerial.h"
#include "Config.h"

//rfm69 and spi *************************		

#include "SPI.h"

SPIClass SPI;

int SPI_CHAN = 1;
int Spi_speed = 500000;

std::string DeviceR = "/dev/gpiofreq";

int Print::out;
int Print::DomoticOut;

Print Serial ;

#define MAXS 4096
int pulse[MAXS];

#define  SLEEP_TIME_IN_US 10000l
int CtMs = 0;

#include "rfmPrint.cpp"

std::string  createVirtualSerial(int &fd);

#define CHANGE 1
#define FALLING 2
#define RISING 3

FILE* fp = 0 ;

//rxPin = numero gpio wiring pi
void attachInterrupt(uint8_t rxGpio, void (*)(void), int mode)

{

	std::string Device;
    int rxBcmPin = wpiPinToGpio(rxGpio);
	Device = DeviceR + std::to_string(rxBcmPin) ;
	Serial.printf("opening %s\n", Device.c_str() );

    if (fp !=0)
        fclose(fp);

	//open pulse driver
	fp = fopen(Device.c_str(), "r");
	if (fp == NULL) {Serial.printf("[ERROR] %s device not found - kernel driver must be started or already running !!\n", Device.c_str());		exit(1); 	}
}
void detachInterrupt(uint8_t intNumber)
{
    fclose(fp);
//				fp = fopen(Device.c_str(), "w");
//				if (fp == NULL) {Serial.printf("[ERROR] open %s device not found - kernel driver must be started !!\n", Device.c_str());exit(1);}

}

//#include "ook.ino"
#include "../Arduino/Ook_OSV12/Ook_OSV12.ino"

void readCom()
{
	char reg[10];
	char val[10];
	int Reg, Val;

	byte nbCar = Serial.available();
	while (nbCar != 0)
	{
		{
			byte b = Serial.Read();
			if (b == 'W')
			{
				reg[0] = Serial.Read();
				reg[1] = Serial.Read();
				reg[2] = 0;
				val[0] = Serial.Read();
				val[1] = Serial.Read();
				val[2] = Serial.Read();

				sscanf(reg, "%x", &Reg);
				sscanf(val, "%x", &Val);

				Serial.printf("write %d : %d\n",Reg,Val);

				readListRegs(RegList);
				radio.writeReg(Reg, Val);
				PrintReg(Reg);


			}
			if (b == 'R')
			{
				reg[0] = Serial.Read();
				reg[1] = Serial.Read();
				reg[2] = 0;

				sscanf(reg, "%x", &Reg);
				PrintReg(Reg);


			}


			nbCar--;

		}
	}
}

static int NbPulses=0;
static int DumpPulse = 0 ;

static int rssiCumul   =0;
static int rssiMin  =0;

void UpDatePulseCounter(int count )
{
    static int NbPulse =0;

	NbPulses += count;
	CtMs += (SLEEP_TIME_IN_US/1000l);
	if ((CtMs % 10000L) == 0)
	{
		NbPulse = NbPulses / 10 ;
		NbPulses = 0;
		Serial.printf( " NbPulse %d\n", NbPulse);
		//fflush(stdout);
		//			fprintf(stdout,"NbPulse %d\n", NbPulsePerSec);
		//			easy->initPin();
		//			easy->setSwitch(1, 0x55, 1);    // turn on device 0
	    if ((CtMs / 10000L) == 1){
         PrintReg(REG_LNA );
         PrintReg(REG_OOKPEAK );
         PrintReg(REG_OOKAVG );
         PrintReg(REG_OOKFIX );
        }
	}
    //chaque sec
	if ((CtMs % 1000L) == 0)
	{
		static int lastrssi=0;
		int rssi = radio.readRSSI();
		if ( abs(lastrssi - rssi) > 2 )
		{
//			Serial.printf("rssi:%d NbPulse %d %d\n", rssi, NbPulse,NbPulses);
		}
		lastrssi = rssi;
        rssiCumul += rssi;
	}
    //chaque min
	if ((CtMs % (1000L*60)) == 0)
	{
        rssiMin = rssiCumul / 60 ;
		{
			Serial.printf("rssi:%d NbPulse %d %d\n", rssiMin, NbPulse );
		}
        rssiCumul = 0 ;
	}
}

int rssiGetAverage()
{
#ifdef RFM69_ENABLE
      return (rssiMin);
#endif
}

int ook_rpi_read_drv(int rxPin, int txPin , int ledpin, int reportType ,  int dumpPulse )
{
    setReportType(reportType);
    Setup( rxPin,  txPin , ledpin,"OTIO ;OOK ;HAGER ;HOMEEASY ;MD230 ;RUBICSON;HIDEKI ;RAIN ;");

	Serial.printf("running\n" );
	while (1) {
		int count = 0;
		count = fread(pulse, 4, 2048, fp);
		UpDatePulseCounter(count);
		if (count > 0)
		{
			for (int i = 0; i < count; i++)
			{
                if(DumpPulse) 
                {
                    static int nbpulses = 0;
                    Serial.printf("%d,", pulse[i]);
                    if ((nbpulses++ % 32)==0)
                        Serial.printf("\n" );
                }
                Loop(pulse[i] );
			}
		}
        else
            Loop(0 );

		usleep(SLEEP_TIME_IN_US);
	}
	fclose(fp);
	return 0;
}

int main(int argc , char** argv)
{
	int debug = 0xff;
    int reportType = 0 ;
	int txPin = 5 ;
	int rxPin = 5;

    int iarg = 0 ;
    std::string sreportType = "";
    FILE* fout;

    Serial.out = fileno(stdout);
    for (iarg = 1 ; iarg<argc;)
    {
        char  swc   = argv[iarg++][1] ;
        char* copt  = argv[iarg++]   ;

        switch (swc)
        {
            //-tx 5
        case 't' :
                txPin = atoi(copt);
            break;
            //-rx 5
        case 'r' :
                rxPin = atoi(copt);
            break;

            //-output  [serial domotic  verbose]
        case 'o' :
                if (copt[0] == 's')       { reportType += REPORT_SERIAL  ;  /* sreportType += "SERIAL " ; */ }
                else  if (copt[0] == 'd') { reportType += REPORT_DOMOTIC ;  /* sreportType += "DOMOTIC "; */ }
                else if (copt[0] == 'v')  { reportType += SERIAL_DEBUG   ;  /* sreportType += "DEBUG "  ; */ }
                else
                    Serial.printf("invalid option : output  [serial domotic  verbose] \n");
            break;
            //-dump [0 1] 
        case 'd' :
                DumpPulse = atoi(copt);
            break;
        case 'f' :
             fout = fopen(copt,"w");  
             if (fout <=0){
                 Serial.printf("error open file %s\n",copt);
             }
             else
                 Serial.out = fileno(fout);
            break;
        default :
                    Serial.printf("invalid option : %c\n", swc );
            break;

        }
    }
    Serial.printf(" -tx 5 -rx 5 -output  [serial domotic  verbose] -dump [0 1] -file ook.log\n");

    if ( reportType == 0 )
            reportType = REPORT_TYPE;
     if ( reportType & REPORT_SERIAL  )  sreportType += "SERIAL "; 
     if ( reportType & REPORT_DOMOTIC )  sreportType += "DOMOTIC ";
     if ( reportType & SERIAL_DEBUG   )  sreportType += "DEBUG " ; 

	Serial.printf("rxPin:%d txPin:%d dumpPulse :%d reportType:%d = %s\n", rxPin,txPin, DumpPulse , reportType , sreportType.c_str());

	//power led sur gpio
	//system("echo gpio | sudo tee /sys/class/leds/led1/trigger");

	if (wiringPiSetup() == -1)	{Serial.printf("[ERROR] failed to initialize wiring pi");exit(1);	}

	//create virtual tty
	
	std::string serial;
	serial = createVirtualSerial(Serial.DomoticOut);
    if (serial.size())
	    Serial.printf("create virtual serial %s \n", serial.c_str());
    else
	    Serial.printf("error opening virtual serial \n");

	//init SPI
	if (SPI.Setup(SPI_CHAN, Spi_speed))
		Serial.printf( "failed to open the SPI bus: ");

	ook_rpi_read_drv(rxPin,txPin, 0, reportType,  dumpPulse);
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