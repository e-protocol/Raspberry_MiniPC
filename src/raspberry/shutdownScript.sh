#! /bin/bash
#   Exports pin to userspace
echo "21" > /sys/class/gpio/export                  

# Sets pin 21 as an output
echo "out" > /sys/class/gpio/gpio21/direction

# Sets pin 21 to high
echo "1" > /sys/class/gpio/gpio21/value
