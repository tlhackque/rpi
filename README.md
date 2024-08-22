# Raspberry Pi tools

## revinfo

`bash` script to decode RPi revision codes

## rtc-ctl

Support for the DS1302 Real-Time Clock.  User mode, supports
all functions - including calibration, setting the system time,
setting the RTC, and accessing the clocks registers.

Compatible with `hwclock` commands for easy replacment in system
scripts.

## gpstime

Support for the MK3333/3339 GPS module, used by Adafruit GPS

Allows basic configuration & setting system time from the module's
RTC before NTP/network is available.
