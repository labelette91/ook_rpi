//file simulate Arduino Serial Print 
#ifndef Print_h
#define Print_h

#include <stdio.h> // for int
#include <string.h> // for int

#include "deftype.h"
#include <unistd.h>
#include <errno.h>
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

int serialAvailable(int fd);


class Print
{
  public:
	static  int out;
	static  int DomoticOut;

	static int PRINT (unsigned long mes , int base , bool lf)
	{
			switch (base) {
					case BIN : dprintf(out,"%d",mes) ;
					break;
					case OCT:  dprintf(out, "%o",mes) ;
					break;
					case DEC : dprintf(out, "%d",mes) ;
					break;
					case HEX : dprintf(out, "%X",mes) ;
					break;
					default  : dprintf(out, "%d",mes) ;
					break;
			}
			if (lf)dprintf(out,"\n") ;
			return 1;
	}


	static int Write(void* pbuffer, int size) {
		uint8_t* buffer = (uint8_t*)pbuffer;
		//log ascii
		dprintf(out, "WR:");for (int i = 0; i < size; i++) dprintf(out, "%02X", buffer[i] ); dprintf(out, "\n");

		int err = write(DomoticOut, pbuffer, size );
		if (err <= 0)
		{
			printf("error %d writing TTY : %d \n", err, strerror(errno));
		}
	

		return size;
	}


    static int print(const char mes [] )                     {return dprintf(out, "%s",mes) ; };
    static int print(char mes )                              {return dprintf(out, "%c",mes) ; };
    static int print(unsigned char mes , int base = DEC)     {return PRINT(mes,base,false) ; };
    static int print(int mes , int base = DEC)               {return PRINT(mes,base,false) ; };
    static int print(unsigned int mes, int base = DEC)       {return PRINT(mes,base,false) ; };
    static int print(long mes , int base = DEC)              {return PRINT(mes,base,false) ; };
    static int print(unsigned long mes , int base = DEC)     {return PRINT(mes,base,false) ; };
    static int print(double mes , int base = 2)              {return dprintf(out, "%f",mes) ; };
    static int println(const char mes[])                     {return dprintf(out, "%s\n",mes) ; };
    static int println(char mes )                            {return dprintf(out, "%c\n",mes) ; };
    static int println(unsigned char mes , int base = DEC)   {return PRINT(mes,base,true) ; };   
    static int println(int mes , int base = DEC)             {return PRINT(mes,base,true) ; };   
    static int println(unsigned int mes , int base = DEC)    {return PRINT(mes,base,true) ; };   
    static int println(long mes , int base = DEC)            {return PRINT(mes,base,true) ; };   
    static int println(unsigned long mes , int base  = DEC)  {return PRINT(mes,base,true) ; };   
    static int println(double mes , int base = 2)            {return dprintf(out, "%f\n",mes) ; };
    static int println(void)								 {return dprintf(out, "\n") ; };

	static int available() 
	{ 
		int nb =  serialAvailable(DomoticOut) ;
//		if (nb) dprintf(out, "NB:%d ", nb );
		return nb;
	}
	static char Read() 
	{ 
		char inputbyte;
		read(DomoticOut, &inputbyte, 1);
//		dprintf(out, "%02X ", inputbyte);

		return inputbyte ;
	}

	static int Read(void* const pbuffer, unsigned const buffer_size)
	{
		int psize = read(DomoticOut, pbuffer, buffer_size);
		char* buffer = (char*)pbuffer;
//		for (int i = 0; i < psize; i++) dprintf(out, "%02X", buffer[i]); dprintf(out, "\n");

		return psize;
	}
};






extern Print Serial ;

#endif