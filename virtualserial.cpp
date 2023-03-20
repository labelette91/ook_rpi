/**
 * Serial port example.
 * Compile with: g++ main.cpp -lpthread -o main
 * 
 * Cymait http://cymait.com
 **/

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#define BUFFER_SIZE 128
#define BAUDRATE    B9600
#include <dirent.h>
#include <unistd.h>
#include <sys/ioctl.h>

void usage(char* cmd) {
    std::cerr << "usage: " << cmd << " slave|master [device, only in slave mode]" << std::endl;
    exit(1);
}

void* reader_thread(void* pointer) {
    int fd = (int)pointer;
    char inputbyte;
    while (read(fd, &inputbyte, 1) == 1) {
        printf("%d ",inputbyte );
        //std::cout << inputbyte;
        std::cout.flush();
    }

    return 0;
}

std::string readLink(std::string linkName)
	{
  char buf[128];
  memset(buf,0,sizeof(buf));
  std::string link = "" ;
  	
  if (readlink(linkName.c_str(), buf, sizeof(buf)) < 0)
        perror("readlink() error");
      else 
      	link = std::string (buf);
return link;      	
}
//find if a serial link exist pointed to ptsX
// return 2 = link already exist
// return 0 = link already exist but point on another ptsName
// 1        = link not exits

int findSerial(std::string serial , std::string ptsX )
{

DIR *d=NULL;
	
// printf("%s\n",serial.c_str() );

	d=opendir("/dev");
	if (d != NULL)
	{
		struct dirent *de=NULL;
		// Loop while not NULL
		while ((de = readdir(d)))
		{
			// Only consider symbolic links
      if (de->d_type == DT_LNK)
      {
				std::string fname = de->d_name;
			  std::string link = readLink("/dev/" + fname) ;
//				printf("f : %s : --> %s \n", fname.c_str(),link.c_str() );
				
        if (fname == serial  )
        {
		    	closedir(d);
        	if (link == ptsX  )
        		return 2;
        	else
        		return 0;
        }

			}
		}
		closedir(d);
	}	
 	return 1 ;
}

//creer le lien /dev/serialxx pointant sur ptsserial /dev/pts/n
std::string  createSerialLink(std::string ptsserial )
{
//	for (int i=1;i<10;i++)
	{
//	  std::string serial = "serial"+std::to_string(i) ;
	  std::string serial = "serialRFX" ;
	  	
		int res =  findSerial(serial , ptsserial );
//  	printf("res : %d\n", res);
		// return 0 = link already exist but point on another ptsName
		if (res==0)
		{
			//del link
			serial = "/dev/"+serial ;
			std::string cmd =  "rm " +  serial  ;			
			int err = system(cmd.c_str() ); 
  			if (err)
				printf("error remove  link  %s : %d\n", serial.c_str() , err);
		}
		if (res>=1)
		{
			serial = "/dev/"+serial ;
			if (res==1)
			{
				  std::string cmd =  "sudo ln -s " +  ptsserial + " " + serial  ;			
				  //system("ls -lh >/dev/null 2>&1"); 
				  int err = system(cmd.c_str() ); 
  				  printf("create link  %s %s : %d\n", ptsserial.c_str() ,serial.c_str() , err);
				  
			}	
			return serial;
		}
	}	
	return "";
}

std::string createVirtualSerial(int &fd )
{
fd = 0;

fd = open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK );
if (fd == -1) {
//	std::cerr << "error opening file" << std::endl;
	return "";
}
grantpt(fd);
unlockpt(fd);

char* pts_name = ptsname(fd);
//std::cerr << "ptsname: " << pts_name << std::endl;

std::string devSerial = createSerialLink(pts_name);
std::cerr << "ptsname: " << pts_name <<  " SerialName: " << devSerial << std::endl;

	/* serial port parameters */
	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	struct termios oldtio;
	tcgetattr(fd, &oldtio);

	newtio = oldtio;
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	tcflush(fd, TCIFLUSH);

	cfsetispeed(&newtio, BAUDRATE);
	cfsetospeed(&newtio, BAUDRATE);
	tcsetattr(fd, TCSANOW, &newtio);
	return devSerial;
}



int createVirtualReadThread (void) {

		int fd;
	std::string serial = createVirtualSerial( fd);
		
    /* read from stdin and send it to the serial port */
    char c;
    while (true) {
        std::cin >> c;
        write(fd, &c, 1);
    }

    close(fd);
    return 0;
}

int serialAvailable(int fd)
{
int bytes_available;
int	retval = ioctl(fd, FIONREAD, &bytes_available);
		if (retval < 0) {
			perror("FIONREAD ioctl failed\n");
			return -1;
		}	
		return bytes_available;
}
