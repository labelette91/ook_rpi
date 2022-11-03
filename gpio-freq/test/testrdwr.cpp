/* ===================================================
C code : test.cpp
* ===================================================
*/

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

 std::string DeviceR= "/dev/gpiofreq" ;
int main(int argc, char *argv[])
{
    FILE * fp;
    std::string Device ;
    	int count = 10 ;
    	#define MAXS 128
    	Device = "17";
    	
    if (argc>=2)
//    count =  atoi(argv[1]) ;
    Device =  (argv[1]) ;
//    else

    if (argc>=3)
      count =  atoi(argv[2]) ;

if (count>=MAXS) count = MAXS;

    Device = DeviceR + Device ;
    printf("opening %s  \n",Device.c_str());
    
    fp = fopen(Device.c_str(), "r+");
    if ( fp == NULL ) {
       printf("[ERROR] %s device not found - kernel driver must be started !!\n",Device.c_str());
       exit(1);
    }

//    char buffer[2048];
//    char * bstart = buffer;
    int bufer[MAXS] ;
    for (int i=0;i<count;i++) bufer[i] = (i+1)*100 + ((i+1)&1);

    fwrite (bufer,4,count,fp);

    count = fread(bufer,4,2048,fp);
	
         printf("count %d : ",count);
      if ( count > 0 ) {
         
         for ( int k = 0 ; k < count ; k ++  ) 
         {
        		if ((k%32)==0)printf("\n");
         		printf("%d ",bufer[k] );
         	}
         
       }	

    return 0;

}
