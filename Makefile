CC 	= gcc
CX	= g++
OBJ_DIR = .
WPI_DIR = /tmp

OOK = ../Arduino/Ook_OSV12
LIB = ../Arduino/libraries

#INC = -I/usr/local/include  -I$(OOK) -I . -I $(LIB)/DecodeOOK -I $(LIB)/tfa
INC = -I/usr/local/include  -I$(OOK) -I . -I$(LIB)/DecodeHomeEasy

TRFLAGS = -DTRACECORE433 -DTRACEEVENTMNG -DTRACESINGLETON 
NOTUSEDTRFLAGS = -DTRACE_RCOOK -DSENSORDEBUG 

CFLAGS = -c -MMD
LDFLAGS += -Xlinker --defsym -Xlinker RFRPI_BUILD_DATE=$$(date +'%Y%m%d') -L/usr/local/lib


rfrpi_dir = ./
rfrpi_files = ./ook_rpi.cpp $(OOK)/Oregon.cpp ./virtualserial.cpp $(LIB)/DecodeHomeEasy/DecodeHomeEasy.cpp $(OOK)/Domotic.cpp ./HomeEasyTransmitter.cpp  ./hager.cpp ./RFM69.cpp $(OOK)/fifo.cpp $(OOK)/reportSerial.cpp $(OOK)/DecodeRain.cpp $(OOK)/util.cpp $(OOK)/DecodePwm.cpp $(OOK)/bitstream.h
rfrpi_objects=$(addsuffix .o,$(basename  $(rfrpi_files)))

target_dir = .
target=./rpi_ook

all:  $(target) ook_rpi.o

$(target): $(rfrpi_objects) 
	$(CX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ -lwiringPi -lpthread 
	chmod +x $@

%.o: %.cpp
	@echo compiling $@ : $<
	$(CX) $(CFLAGS) $(INC) $< -o $@ $(TRFLAGS)

clean:
	@rm $(OBJ_DIR)/*.o
	@rm $(target)
ook_rpi.o: ook_rpi.cpp  ../Arduino/Ook_OSV12/Ook_OSV12.ino
	@echo compiling $@ : $<
	$(CX) $(CFLAGS) $(INC) $< -o $@ $(TRFLAGS)
	