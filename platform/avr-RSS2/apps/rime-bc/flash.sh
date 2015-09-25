#!/bin/bash
make bc hex TARGET=avr-RSS2
/usr/bin/stty sane
/usr/sbin/tty_talk -38400 /dev/ttyUSB0 "upgr
"
/usr/bin/avrdude -p m128rfa1 -c avr109 -P /dev/ttyUSB0 -b 38400 -e -U flash:w:bc.hex
