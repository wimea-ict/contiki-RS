#!/bin/bash

make bc hex TARGET=avr-radiosensors

avrdude -p m128rfa1 -c avr109 -P /dev/ttyUSB0 -b 38400 -e -U flash:w:bc.hex
