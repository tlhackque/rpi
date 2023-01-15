# Make rtc-ctl - the DS1302 TOY manager for the RPi
#
# Copyright (C) 2015 Timothe Litt litt at ddot acm dot org
#
# To build:
#  The default build assumes the RTC is connected to the GPIOs below. To change:
#
#    make "RTC_GPIOS=-D CE_PIN=RPI_GPIO_P1_16 -D CK_PIN=RPI_GPIO_P1_15 -D IO_PIN=RPI_GPIO_P1_22"
#
#  Pin names reflect RPi connector names, and vary by RPi version.
#  For details, see rtc-ctl.h
#
#  To omit NTP support (not recommended)
#    make "RTC_OPTIONS=-DCHECK_NTP=0"
#
#  Otherwise, simply
#    make
#    make install

CPPFLAGS=

# Include GPIO overrides

CPPFLAGS+=$(RTC_GPIOS)

# Add any other compile options

CPPFLAGS+=$(RTC_OPTIONS)

# Compiler options

CFLAGS=-g -O2 -Wall -Woverflow -Wstrict-overflow -Wshadow -Wextra -pedantic

LDLIBS=-lrt -lm

LDFLAGS=

# Utilities

INSTALL=install
TAR=tar

# Files packaged in the distribution kit

PACKAGED=README COPYING Makefile rtc_ctl rtc-ctl.c rtc-ctl.cron rtc-ctl.h TOYeval.xls

.PHONY : all

# Build always runs show-config so GPIO config is in front of user

all : rtc-ctl
	./rtc-ctl --show-config

# The utility

rtc-ctl :

# Object depends on makefile and header

rtc-ctl.o : rtc-ctl.c rtc-ctl.h Makefile

# Install

.PHONY : install

install : rtc-ctl rtc_ctl
	$(INSTALL) -p rtc-ctl /sbin
	$(INSTALL) -p rtc_ctl /etc/init.d
	echo "\$time	rtc_ctl" >/etc/insserv.conf.d/rtc_ctl
	[ -f /etc/default/rtc_ctl ] || echo "#DEBUG=yes; SHUTDOWN_ARGS=--no-ntp" >/etc/default/rtc_ctl
	$(INSTALL) -p rtc-ctl.cron /etc/cron.d/hourly
	update-rc.d fake-hwclock disable
	-[ -x /etc/cron.hourly/fake-hwclock ] && chmod -x /etc/cron.hourly/fake-hwclock
	update-rc.d rtc_ctl defaults
	[ -f /usr/share/rtc-ctl/TOYeval.xls ] || $(INSTALL) -p -D TOYeval.xls /usr/share/rtc-ctl/TOYeval.xls

# Install, stripping symbols

.PHONY : install-strip

install-strip : install
	$(INSTALL) -s rtc-ctl /sbin

# Uninstall

.PHONY : uninstall

# Does not uninstall the spreadsheet as it may have user data

uninstall :
	-[ -f /etc/init.d/rtc_ctl ] && rm /etc/init.d/rtc_ctl || true
	-[ -f /sbin/rtc-ctl ] && rm /sbin/rtc-ctl || true
	-[ -f /etc/insserv.conf.d/rtc_ctl ] && rm /etc/insserv.conf.d/rtc_ctl || true
	-[ -f /etc/cron.d/hourly/rtc-ctl.cron ] && rm /etc/cron.d/hourly/rtc-ctl.cron || true
	-update-rc.d rtc_ctl remove
	-update-rc.d fake-hwclock enable
	-[ -r /etc/cron.hourly/fake-hwclock ] && chmod +x /etc/cron.hourly/fake-hwclock || true
	[ -f /usr/share/rtc-ctl/TOYeval.xls ] && echo "/usr/share/rtc-ctl/TOYeval.xls not removed" || true

# Build distribution kit

.PHONY : dist

dist : $(PACKAGED) rtc-ctl
	if [ -z "$(strip $(shell ./rtc-ctl --version))" ] || [ "$(strip $(shell ./rtc-ctl --version))" = "/" ] ; then false; fi
	-[ -d $(strip $(shell ./rtc-ctl --version)) ]  && rm -rf $(strip $(shell ./rtc-ctl --version))/ || true
	mkdir $(strip $(shell ./rtc-ctl --version))
	cp -p $(PACKAGED) $(strip $(shell ./rtc-ctl --version))/
	$(TAR) -czf $(strip $(shell ./rtc-ctl --version)).tar.gz $(strip $(shell ./rtc-ctl --version))/
	rm -rf $(strip $(shell ./rtc-ctl --version))/

# Cleanup

.PHONY : clean

clean :
	-rm rtc-ctl rtc-ctl.o
