#include "serial.h"

Serial::Serial(int baud):_baud(baud)
{
	_handle = -1;
}

Serial::~Serial()
{
    if(_handle >= 0)
        closeSerial();
}

void Serial::closeSerial(void)
{
    if(_handle >= 0)
        close(_handle);

    _handle = -1;
}

void Serial::setBaud(int baud)
{
    if(isOpen())
        return;
        
    _baud = baud;
}

bool Serial::openSerial(std::string deviceName)
{
    struct termios tio;
    struct termios2 tio2;
    _deviceName = deviceName;
    _handle  = open(_deviceName.c_str(),O_RDWR | O_NOCTTY /* | O_NONBLOCK */);

    if(_handle < 0)
        return false;
       
    tio.c_cflag =  CS8 | CLOCAL | CREAD;
    tio.c_oflag = 0;
    tio.c_lflag = 0;       //ICANON;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;     // time out every .1 sec
    ioctl(_handle,TCSETS,&tio);

    ioctl(_handle,TCGETS2,&tio2);
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;
    tio2.c_ispeed = _baud;
    tio2.c_ospeed = _baud;
    ioctl(_handle,TCSETS2,&tio2);
    
    ioctl(_handle,TCFLSH,TCIOFLUSH);//flush buffer
    fcntl(_handle, F_SETFL, 0); //set non-blocking read
    
    return true;
}

bool Serial::writeData(std::string value)
{
    if(!isOpen()) 
        return false;
       
   int rlen = write(_handle,value.c_str(),value.size()); 
   return(rlen == static_cast<int>(value.size()));
}

int Serial::available()
{
    if(!isOpen()) 
        return -1;
    
    int byteLen;
    ioctl(_handle, FIONREAD, &byteLen);
    return byteLen;
}

std::string Serial::readData()
{
    int lenRCV = 0;
    int len = available();
    
    if(len < 1)
        return std::string();
    
    char input[len];
   
    while(lenRCV < len)
    {
       int rlen = read(_handle,&input[lenRCV],len - lenRCV);
       lenRCV += rlen;
    }
    
    ioctl(_handle,TCFLSH,TCIOFLUSH);//flush buffer
    return std::string(&input[0],len);
}
