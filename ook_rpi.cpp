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

byte data[4];

#define MAXS 4096
int pulse[MAXS];
int 	NbPulse;

#ifdef OTIO_ENABLE        
#include "decodeotio.h"
DecodeOTIO Otio(3);
#endif


void PulseLed()
{

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

std::string  createVirtualSerial(int &fd);

int ook_rpi_read_drv( int gpio)
{
	FILE* fp;
	std::string Device;
	std::string serial;


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


	while (1) {
		int count = fread(pulse, 4, 2048, fp);


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
							PulseLed();
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
					PulseLed();
				}
#endif      	

		}
		}
		usleep(10000l);
	}
	fclose(fp);
	return 0;
}




int main()
{
	ook_rpi_read_drv(17);
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

int millis()
{

	long            ms; // Milliseconds
	time_t          s;  // Seconds
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	ms = spec.tv_sec * 1000 + (spec.tv_nsec / 1000000 ); 

	return ms;
}
int micros()
{
	return 0;
}

#ifdef WIN32

int usleep(int)
{
	return 0;

}
#endif