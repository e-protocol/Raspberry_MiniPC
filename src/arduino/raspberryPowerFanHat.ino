/* Device: raspberryPowerFanHat
 * Board: Arduino Nano, soldering board
 * Modules: INA219, Custom Power Supply, 
 * Custom Button Switch, OLED 0.96 Display SSD1306 128x64,
 * Raspberry pi 4 8GB
 * Details: 
 * 1) Controls Fan speed via PWM pin 9 at speed 20-100%
 * 2) Power Supply 5V 10A to Raspberry
 * 3) Button Power On/Off switch
 * 4) Arduino cut the power after 5 seconds 
 * recieve raspberry shutdown signal
 * 5) Measure rasbperry power supply
 * 6) View on Display rasbperry CPU temperature
 * and power supply data
 * 7) Setting with PWM ~31.25 KHz
 * native PWM is ~500 Hz
 * 
 * 0x3C - I2C address Display
 * 0x40 - I2C address Ina219
 * SDA pin on A4
 * SCL pin on A5
 * Created by E-Protocol
 * 
 * Raspberry Power Supply
 * Normal 5.1-5.5V
 * Stable work on 5.25V 
 */

//display 0.96 128x64 SSD1306
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);
const int charCountLine = 10; //number of chars per display line
const int fontWidth = 7, fontHeight = 14;
const unsigned char fanImg[] PROGMEM =
{
  0x07, 0x00, 0x0f, 0x80, 0x0f, 0x80, 0x07, 0x80, 0x03, 0x98, 0x78, 0x3c, 0xfb, 0x7c, 0xfb, 0x7c, 
  0xf0, 0x78, 0x67, 0x00, 0x07, 0x80, 0x07, 0xc0, 0x07, 0xc0, 0x03, 0x80
};

//INA219 volt-amp meter
#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 meterVA;
float volt = 0.0, amp = 0.0, power = 0.0;
unsigned long prevOutputTime = 0;
const int printInterval = 2000;
float cpuTemp = 0.0;

//fan control and shutdown
#define PIN_PWM 9 //fan pwm pin
#define READ_INTERVAL 20 //read Serial every 20ms
#define PIN_LISTENER 10 //pin to Raspberry GPIO21
#define PIN_SHUTDOWN 11 //pin to button MOSFET
#define SHUTDOWN_TIME 5000
#define STRING_SIZE 16
unsigned long prevShutdownTime = 0;
bool isShutdown = false;
unsigned long previousRead = 0;
const int maxPWM = 155, minPWM = 135;
char inputArr[STRING_SIZE]; //input string
unsigned int endIndex = 0;
int fanSpeed = 30, fanStep = 10;
const int minFanRange = 0,
maxFanRange = 100, minFanSpeed = 30; //speed in %

float getMapVal(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calcValues()
{
    float busvoltage = meterVA.getBusVoltage_V();
    float shuntvoltage = meterVA.getShuntVoltage_mV();
    volt = busvoltage + (shuntvoltage / 1000.0);
    amp = meterVA.getCurrent_mA() / 1000.0;
    
    if(isinf(amp) || isnan(amp))
        amp = 0.0;

    if(amp < 0.0)
        amp *= -1;

    if(volt < 0.0)
        volt *= -1;

    power = amp * volt; //meterVA.getPower_mW() / 1000.0;
}

void printOutput()
{
    u8g.firstPage();
    
    do 
    {
        uint8_t yPos = fontHeight + 1;
        uint8_t xPos = 0;
        uint8_t ySpace = yPos;
        String data = "";
      
        u8g.setPrintPos(xPos,yPos);
        data = String(volt,2);
        u8g.print("V" + insertSpaces(calcCharCount(data)) 
        + data);
        yPos += ySpace;
        
        u8g.setPrintPos(xPos,yPos);
        data = String(amp,2);
        u8g.print("A" + insertSpaces(calcCharCount(data)) 
        + data);
        yPos += ySpace;
        
        u8g.setPrintPos(xPos,yPos);
        data = String(power,2);
        u8g.print("W" + insertSpaces(calcCharCount(data)) 
        + data);
        yPos += ySpace;

        //print cpu temperature
        u8g.setPrintPos(xPos,yPos);
        u8g.print((char)0xB0);
        xPos = fontWidth;
        u8g.setPrintPos(xPos,yPos);
        data = String(cpuTemp,1);
        u8g.print("C" + insertSpaces(calcCharCount(data) - 1)  
        + data);
        xPos = 0;
        yPos += 2;

        //draw bitmap image of fan
        u8g.drawBitmapP(xPos, yPos, 2 , 14, fanImg);
        xPos = fontWidth;
        yPos += ySpace - 2;
        u8g.setPrintPos(xPos,yPos);
        data = String(fanSpeed) + "%";
        u8g.print(insertSpaces(calcCharCount(data)) + data);
    } while (u8g.nextPage());
}

String insertSpaces(unsigned int count)
{
    String str = "";

    for(int k = 0; k < count; k++)
        str += " ";

    return str;
}

int calcCharCount(String str)
{
    return charCountLine - sizeof(str);
}

bool compareChars(char *first, char *second)
{
    if(sizeof(first) != sizeof(second))
        return false;

    for(int k = 0; k < sizeof(first); k++)
        if(first[k] != second[k])
            return false;

    return true;
}
  
void parseInput()
{
    if(Serial.available() == 0)
        return;
     
    while(Serial.available() && 
          endIndex < STRING_SIZE)
    { 
        inputArr[endIndex] = Serial.read();
        endIndex++;
    }

    //clear if array is full
    //else clear after command parsed
    if(inputArr[--endIndex] != '\n' && 
       endIndex == STRING_SIZE - 1)
    {
        clearSerial();
        return;
    }

    processRaspberry();
    processFanSpeed();
    clearSerial();
}

void processFanSpeed()
{
    fanSpeed = getMapVal(cpuTemp, minFanSpeed, maxFanRange / 2, 
               minFanRange, maxFanRange); //cpu 30-50 C, fan 0-100%
    fanSpeed = fanSpeed / fanStep * fanStep; //floor fanSpeed

    if(fanSpeed < minFanSpeed)
        fanSpeed = minFanSpeed;
    else if(fanSpeed > maxFanRange)
        fanSpeed = maxFanRange;

    int pwmVal = getMapVal(fanSpeed, minFanRange, maxFanRange, minPWM, maxPWM);
    analogWrite(PIN_PWM, pwmVal);
}

void processRaspberry()
{
    if(compareChars(inputArr,"req dev"))
    {
        Serial.println("powerSupply");
        return;
    }

    float val = atof(inputArr); //char* to float
    
    if(val > 120.0)
        cpuTemp = 120.0;
    else if(val >= 0.0)
        cpuTemp = val;
}

void clearSerial()
{
    for(int k = 0; k < STRING_SIZE; k++)
        inputArr[k] = ' ';

    endIndex = 0;
}

void checkShutdown(unsigned long currentTime)
{
    if(!isShutdown && 
       digitalRead(PIN_LISTENER) == HIGH)
    {
        isShutdown = true;
        prevShutdownTime = currentTime;
    }

    if(isShutdown && prevShutdownTime > 0 &&
       currentTime - prevShutdownTime > SHUTDOWN_TIME)
    {
      digitalWrite(PIN_SHUTDOWN, HIGH);
      isShutdown = false;
      Serial.println(currentTime - prevShutdownTime);
      delay(100);
    }
}

void setup() 
{
    meterVA.begin();
    u8g.setFont(u8g_font_7x14);
    u8g.setRot90(); //rotate screen position
    
    pinMode(PIN_PWM, OUTPUT);
    //set PWM 31.4 KHz Nano
    //Pins D9 and D10 - 31.4 KHz Timer 1
    TCCR1A = 0b00000001;  // 8bit
    TCCR1B = 0b00000001;  // x1 phase correct
    Serial.begin(115200);
    //start fan on start up
    int pwmVal = getMapVal(fanSpeed, minFanRange, maxFanRange, minPWM, maxPWM);
    analogWrite(PIN_PWM, pwmVal);
    //shutdown
    pinMode(PIN_LISTENER, INPUT);
    pinMode(PIN_SHUTDOWN, OUTPUT);
    digitalWrite(PIN_SHUTDOWN, LOW);
}

void loop() 
{
    unsigned long currentTime = millis();
    checkShutdown(currentTime);
    
    //read Serial
    if(currentTime - previousRead > READ_INTERVAL)
    {
        previousRead = currentTime;
        parseInput();
    }

    //write Serial
    if(currentTime - prevOutputTime > printInterval)
    {
        prevOutputTime = currentTime;
        //ina219 data
        calcValues();
        printOutput();
    }
}
