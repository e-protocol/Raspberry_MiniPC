CC=g++
CFLAGS=-c -Wall
LDFLAGS=
SOURCES=cpuTemp.cpp serial.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=cpuTemp

all: $(SOURCES) $(EXECUTABLE) clean
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -f *.c *.o
