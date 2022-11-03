CXXFLAGS += -O2 -Wall -g -pthread

all: test testrd testrdwr

test:  test.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $+ -o $@ 

testrd:  testrd.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $+ -o $@ 

testrdwr:  testrdwr.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $+ -o $@ 

clean:
	$(RM) *.o test
