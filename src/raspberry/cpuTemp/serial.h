#ifndef SERIAL
#define SERIAL 

#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>

class Serial
{
public: 
	Serial(int baud);
	~Serial();
	bool openSerial(std::string deviceName);
	bool isOpen(void) { return _handle >= 0; }
	void closeSerial(void);
	bool writeData(std::string value);
	void setBaud(int baud);
	int available();
	std::string readData();
	
private:
	std::string _deviceName;
	int _baud;
	int _handle;
};

#endif
