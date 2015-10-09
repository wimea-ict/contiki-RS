#!/bin/bash
make bc hex TARGET=avr-RSS2

S_PROG=avr109
#S_PROG=stk500v2

PGM=

stty sane
tty_talk -38400 /dev/ttyUSB0 "upgr
"
avrdude -p m128rfa1 -c $S_PROG -P /dev/ttyUSB0 -b 38400 -e -U flash:w:bc.hex
