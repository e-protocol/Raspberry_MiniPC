********************************************************************************
## Raspberry_MiniPC<br/>
<p float="left">
<img src = "Project Photo/img-1.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-2.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-3.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-4.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-5.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-6.jpg" width = "409" height = "230" />
<img src = "Project Photo/img-7.jpg" width = "723" height = "224" />
<img src = "Project Photo/img-8.jpg" width = "434" height = "214" />
</p>
https://user-images.githubusercontent.com/56933629/200182420-3281cdc3-58ea-40ac-bcb1-8d5b57b517ea.mp4
Created by E-Protocol
https://github.com/e-protocol
<br/>
Specs: Arduino Nano, Raspberry Pi4 8Gb
Description: Introducing raspberry miniPC. This great curcuit gives you: the compactness and DIY
build ability of microcontrollers like Arduino on one hand and on the other the PC's computing 
power and memory resources. But it has lack of some components to call it a miniPC:
- power button to turn ON/OFF
- M2 SSD for 256 Gb as a Hard Disk
- cooling fan to gain a stable work on full CPU load
- power supply for power surge protection.
So I just put it all together in one box. The power supply has the ability to start curcuit and to power off
curcuit on shutdown by OS command. Arduino required to detect shutdown pin HIGH signal and to show on mini 
screen usefull info: fan load, tempereture, curcuit voltage and amperage, so you can easy calculate 
the power demand.
<br/>
********************************************************************************
<br/>
Arduino part
With PWM it controlls fan speed. Read income data from raspberry by UART and print it on mini screen.
The arduino sketch can be found at src/arduino
<br/>
********************************************************************************
<br/>
Raspberry part
startup.sh script runs cpuTemp programm on system boot, this programm reads cpu temperature and 
sends it by COM to Arduino. The source code and make file can be found at src/raspberry/cpuTemp
Just for help the raspberry pin map can be found at Project Photo/img-7.jpg

<br/>
********************************************************************************
<br/>
Power supply
This is a DC-DC 12-5V converter. It requires at least 12V 3A power supply.
The raspberry curcuit itself requires at least 5V for stable work.
The curcuit can be found at Project Photo/img-8.jpg
There is a demonstartion short video of power ON/OFF

NOTE! Under load the voltage goes down, so with help of mulimeter adjust the potentiometer
to get voltage around 5.2-5.4V. The real PC power supply has lots of additional stuff:
- protection from short curcuit
- Big voltage and amperage stabilizer
- 3 supply lines: 12V, 5V, 3.3V with bigger amperage
- auto voltage adjustment for each line to get stable voltage output
- reset function to power off and power up which allows ro restart the system
