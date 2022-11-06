#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "serial.h"

const std::string _deviceName = "powerSupply";

void msleep(int milliSeconds)
{
	std::this_thread::sleep_for(std::chrono::
	milliseconds(milliSeconds));
}

float getTemperature()
{
	//open file with cpu temperature
	std::string fileName = "/sys/class/thermal/thermal_zone0/temp";
	std::ifstream fileStream;
	std::string input;
	
	fileStream.open(fileName);
	fileStream >> input;
	fileStream.close();
	
	float cpuTemp = std::stof(input);
	cpuTemp /= 1000;
	cpuTemp = roundf(cpuTemp * 10) / 10;
	std::cout << "CPU: " << cpuTemp << "°C"<< std::flush;
	return cpuTemp;
}

//find device powerSupply usb
bool findDevice(Serial &serial)
{		
	for(int k = 0; k < 5; k++)
	{
		serial.openSerial("/dev/ttyUSB" + std::to_string(k));
		msleep(10);
		
		if(serial.isOpen())
		{
			int tries = 4;
			
			while(tries > 0)
			{
				serial.writeData("req dev");
				msleep(500);
				std::string input = "";
		
				if(serial.available() > 0)
					input = serial.readData();
					
				
				input.erase(std::remove(input.begin(), input.end(), 
					'\n'),input.end()); //remove '\n' characters
					
				if(input == _deviceName)
				{
					std::cout << input << "\n";
					return true;
				}
				
				tries--;
			}
		}
		
		serial.closeSerial();
	}
	return false;
}

int main()
{
	Serial serial(115200);
	bool isDevice = findDevice(serial);
	
	while(true)
	{				
		std::cout << "\r";
		std::string data = std::to_string(getTemperature());
		
		if(isDevice)
			serial.writeData(data);
			
		msleep(1000);
	}
	
	return 0;
}
