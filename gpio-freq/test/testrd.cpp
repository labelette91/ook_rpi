/* ===================================================
C code : test.cpp
* ===================================================
*/

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define WR_VALUE _IOW('a','a',int32_t*)
#define RD_VALUE _IOR('a','b',int32_t*)

 std::string DeviceR= "/dev/gpiofreq" ;
int main(int argc, char *argv[])
{
    FILE * fp;
    std::string Device ;
    	int dur=0;
    	
//    if (argc>=2)
//    Device =  argv[1] ;
//    else
    if (argc>=2)
    	dur  = atoi( argv[1]) ;

//    Device = DeviceR + "17" ; // GPIO 0
    Device = DeviceR + "24" ; // GPIO 5
	
    printf("opening %s  dur:%d\n",Device.c_str(),dur);
    
    fp = fopen(Device.c_str(), "r");
    if ( fp == NULL ) {
       printf("[ERROR] %s device not found - kernel driver must be started !!\n",Device.c_str());
       exit(1);
    }

//    char buffer[2048];
#define MAXS 4096
#define NBCT 500

    int pulse[MAXS];
    int ct=NBCT;
    int nbP = 0 ;
    int32_t value ;

//    char * bstart = buffer;
    while (ct--) {

    	ioctl(fileno(fp), RD_VALUE, (int32_t*) &value);

//       	printf("\nioctl %d ",  value  );

       	int count = fread(pulse,4,2048,fp);

	nbP += count ;
			
       if ( count > 0 ) {
         printf("count %d : ",count);
         
         for ( int k = 0 ; k < count ; k ++  ) 
         {
        		if ((k%32)==0)printf("\n");
         		printf("%d ",pulse[k] );
         	}
         
       }
       usleep(100000l);
    }
 		printf("\n Pulse:%d = %d per sec\n",nbP , nbP/ (NBCT/100));
    fclose(fp);
    return 0;

}
