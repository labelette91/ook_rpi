CC 	= gcc
CX	= g++
OBJ_DIR = .
WPI_DIR = /tmp

INC = -I/usr/local/include -I .
TRFLAGS = -DTRACECORE433 -DTRACEEVENTMNG -DTRACESINGLETON 
NOTUSEDTRFLAGS = -DTRACE_RCOOK -DSENSORDEBUG 

CFLAGS = -c -MMD
LDFLAGS += -Xlinker --defsym -Xlinker RFRPI_BUILD_DATE=$$(date +'%Y%m%d') -L/usr/local/lib

rfrpi_dir = ./
rfrpi_files = ./ook_rpi.cpp ./oregon.cpp ./virtualserial.cpp ./domotic.cpp
rfrpi_objects=$(addsuffix .o,$(addprefix $(OBJ_DIR)/,$(basename $(notdir $(rfrpi_files))))) 

target_dir = .
target_files+= ook_rpi.cpp
target_objects=$(addsuffix .o,$(addprefix $(OBJ_DIR)/,$(basename $(notdir $(target_files))))) 
target=./ook_rpi


all:  $(target) 

$(target): $(target_objects) $(rfrpi_objects) 
	$(CX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ -lwiringPi -lpthread 
	chmod +x $@

$(OBJ_DIR)/%.o: $(rfrpi_dir)/%.cpp $(rfrpi_dir)/%.h
	@echo '----------------------------------'
	@echo compiling $@
	$(CX) $(CFLAGS) $(INC) $< -o $@ $(TRFLAGS)
	@echo '-------------'
	
$(OBJ_DIR)/%.o: $(target_dir)/%.cpp
	@echo '----------------------------------'
	@echo compiling $@
	$(CX) $(CFLAGS) $(INC) $< -o $@ $(TRFLAGS)
	@echo '-------------'

clean:
	@rm $(OBJ_DIR)/*.o
	@rm $(target)
	