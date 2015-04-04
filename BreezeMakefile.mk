# Makefile for Breeze Project
# old flags : -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD 
# /home/hadjsalah/Software/arduino-1.6.3/hardware/arduino/avr

BOARD := mega2560

# default arduino software directory, check software exists
ifndef ARDUINODIR
ARDUINODIR := /home/hadjsalah/Software/arduino-1.6.3
endif
ifeq "$(wildcard $(ARDUINODIR)/hardware/arduino/avr/boards.txt)" ""
$(error ARDUINODIR is not set correctly; arduino software not found)
endif

# default arduino version
ARDUINOCONST ?= 100

# default path for avr tools
AVRTOOLSPATH ?= $(subst :, , $(PATH)) $(ARDUINODIR)/hardware/tools \
	$(ARDUINODIR)/hardware/tools/avr/bin

# default path to find libraries
LIBRARYPATH ?= libraries libs $(SKETCHBOOKDIR)/libraries $(ARDUINODIR)/libraries

# default serial device to a poor guess (something that might be an arduino)
SERIALDEVGUESS := 0
ifndef SERIALDEV
SERIALDEV := $(firstword $(wildcard \
	/dev/ttyACM? /dev/ttyUSB? /dev/tty.usbserial* /dev/tty.usbmodem*))
SERIALDEVGUESS := 1
endif

# no board?
ifndef BOARD
ifneq "$(MAKECMDGOALS)" "boards"
ifneq "$(MAKECMDGOALS)" "clean"
$(error BOARD is unset.  Type 'make boards' to see possible values)
endif
endif
endif

# obtain board parameters from the arduino boards.txt file
BOARDSFILE := $(ARDUINODIR)/hardware/arduino/avr/boards.txt
readboardsparam = $(shell sed -ne "s/$(BOARD).$(1)=\(.*\)/\1/p" $(BOARDSFILE))
BOARD_BUILD_MCU := atmega2560
BOARD_BUILD_FCPU := 16000000L
BOARD_BUILD_VARIANT := mega
BOARD_UPLOAD_SPEED := 115200
BOARD_UPLOAD_PROTOCOL := wiring
BOARD_USB_VID := $(call readboardsparam,build.vid)
BOARD_USB_PID := $(call readboardsparam,build.pid)
BOARD_BOOTLOADER_UNLOCK := 0x3F
BOARD_BOOTLOADER_LOCK := 0x0F
BOARD_BOOTLOADER_LFUSES := 0xFF
BOARD_BOOTLOADER_HFUSES := 0xD8
BOARD_BOOTLOADER_EFUSES := 0xFD
BOARD_BOOTLOADER_PATH := $(call readboardsparam,bootloader.path)
BOARD_BOOTLOADER_FILE := $(call readboardsparam,bootloader.file)

# obtain preferences from the IDE's preferences.txt
PREFERENCESFILE := $(firstword $(wildcard \
	$(HOME)/.arduino/avr/preferences.txt $(HOME)/Library/Arduino/preferences.txt))
ifneq "$(PREFERENCESFILE)" ""
readpreferencesparam = $(shell sed -ne "s/$(1)=\(.*\)/\1/p" $(PREFERENCESFILE))
SKETCHBOOKDIR := $(call readpreferencesparam,sketchbook.path)
endif

# invalid board?
ifeq "$(BOARD_BUILD_MCU)" ""
ifneq "$(MAKECMDGOALS)" "boards"
ifneq "$(MAKECMDGOALS)" "clean"
$(error $(BOARD) is invalid.  Type 'make boards' to see possible values)
endif
endif
endif

# auto mode?
INOFILE := $(wildcard *.ino *.pde)
ifdef INOFILE
ifneq "$(words $(INOFILE))" "1"
$(error There is more than one .pde or .ino file in this directory!)
endif

# automatically determine sources and targeet
TARGET := $(basename $(INOFILE))
SOURCES := $(INOFILE) \
	$(wildcard *.c *.cc *.cpp *.C) \
	$(wildcard $(addprefix util/, *.c *.cc *.cpp *.C)) \
	$(wildcard $(addprefix utility/, *.c *.cc *.cpp *.C))

# automatically determine included libraries
LIBRARIES := $(filter $(notdir $(wildcard $(addsuffix /*, $(LIBRARYPATH)))), \
	$(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(SOURCES)))

endif

# software
findsoftware = $(firstword $(wildcard $(addsuffix /$(1), $(AVRTOOLSPATH))))
CC := $(call findsoftware,avr-gcc)
CXX := $(call findsoftware,avr-g++)
LD := $(call findsoftware,avr-ld)
AR := $(call findsoftware,avr-ar)
OBJCOPY := $(call findsoftware,avr-objcopy)
AVRDUDE := $(call findsoftware,avrdude)
AVRSIZE := $(call findsoftware,avr-size)

# directories
ARDUINOCOREDIR := $(ARDUINODIR)/hardware/arduino/avr/cores/arduino
LIBRARYDIRS := $(foreach lib, $(LIBRARIES), \
	$(firstword $(wildcard $(addsuffix /$(lib), $(LIBRARYPATH)))))
LIBRARYDIRS += $(addsuffix /utility, $(LIBRARYDIRS))

# files
TARGET := $(if $(TARGET),$(TARGET),a.out)
OBJECTS := $(addsuffix .o, $(basename $(SOURCES)))
DEPFILES := $(patsubst %, .dep/%.dep, $(SOURCES))
ARDUINOLIB := $(ARDUINODIR)/hardware/libraries # TODO HERE
ARDUINOLIBOBJS := $(foreach dir, $(ARDUINOCOREDIR) $(LIBRARYDIRS), \
	$(patsubst %, .lib/%.o, $(wildcard $(addprefix $(dir)/, *.c *.cpp))))
BOOTLOADERHEX := $(addprefix \
	$(ARDUINODIR)/hardware/arduino/avr/bootloaders/$(BOARD_BOOTLOADER_PATH)/, \
	$(BOARD_BOOTLOADER_FILE))

# avrdude confifuration
ifeq "$(AVRDUDECONF)" ""
ifeq "$(AVRDUDE)" "$(ARDUINODIR)/hardware/tools/avr/bin/avrdude"
AVRDUDECONF := $(ARDUINODIR)/hardware/tools/avr/etc/avrdude.conf
else
AVRDUDECONF := $(wildcard $(AVRDUDE).conf)
endif
endif

# flags
CPPFLAGS += -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections
CPPFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -std=c++11
CPPFLAGS += -mmcu=$(BOARD_BUILD_MCU)
CPPFLAGS += -DF_CPU=$(BOARD_BUILD_FCPU) -DARDUINO=$(ARDUINOCONST)
# CPPFLAGS += -DUSB_VID=$(BOARD_USB_VID) -DUSB_PID=$(BOARD_USB_PID)
CPPFLAGS += -I. -Iutil -Iutility -I $(ARDUINOCOREDIR)
CPPFLAGS += -I $(ARDUINODIR)/hardware/arduino/avr/variants/$(BOARD_BUILD_VARIANT)/
CPPFLAGS += $(addprefix -I , $(LIBRARYDIRS))
CPPDEPFLAGS = -MMD -MP -MF .dep/$<.dep
CPPINOFLAGS := -x c++ -include $(ARDUINOCOREDIR)/Arduino.h
AVRDUDEFLAGS += $(addprefix -C , $(AVRDUDECONF)) -DV
AVRDUDEFLAGS += -p $(BOARD_BUILD_MCU) -P $(SERIALDEV)
AVRDUDEFLAGS += -c $(BOARD_UPLOAD_PROTOCOL) -b $(BOARD_UPLOAD_SPEED)
LINKFLAGS += -Os -Wl,--gc-sections -mmcu=$(BOARD_BUILD_MCU)

# figure out which arg to use with stty (for OS X, GNU and busybox stty)
STTYFARG := $(shell stty --help 2>&1 | \
	grep -q 'illegal option' && echo -f || echo -F)

# include dependencies
ifneq "$(MAKECMDGOALS)" "clean"
-include $(DEPFILES)
endif

# default rule
.DEFAULT_GOAL := all

#_______________________________________________________________________________
#                                                                          RULES

.PHONY:	all target upload clean boards monitor size bootloader

all: target

target: $(TARGET).hex

upload: target
	@echo "\nUploading to board..."
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
ifeq "$(BOARD_BOOTLOADER_PATH)" "caterina"
	stty $(STTYFARG) $(SERIALDEV) speed 1200
	sleep 1
else
	stty $(STTYFARG) $(SERIALDEV) hupcl
endif
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(TARGET).hex:i

clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET).elf $(TARGET).hex $(ARDUINOLIB) *~
	rm -rf .lib .dep

boards:
	@echo "Available values for BOARD:"
	@sed -nEe '/^#/d; /^[^.]+\.name=/p' $(BOARDSFILE) | \
		sed -Ee 's/([^.]+)\.name=(.*)/\1            \2/' \
			-e 's/(.{12}) *(.*)/\1 \2/'

monitor:
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test -n `which screen` || { \
		echo "error: can't find GNU screen, you might need to install it." >&2 \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
	screen $(SERIALDEV)

size: $(TARGET).elf
	echo && $(AVRSIZE) --format=avr --mcu=$(BOARD_BUILD_MCU) $(TARGET).elf

bootloader:
	@echo "Burning bootloader to board..."
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
	stty $(STTYFARG) $(SERIALDEV) hupcl
	$(AVRDUDE) $(AVRDUDEFLAGS) -U lock:w:$(BOARD_BOOTLOADER_UNLOCK):m
	$(AVRDUDE) $(AVRDUDEFLAGS) -eU lfuse:w:$(BOARD_BOOTLOADER_LFUSES):m
	$(AVRDUDE) $(AVRDUDEFLAGS) -U hfuse:w:$(BOARD_BOOTLOADER_HFUSES):m
ifneq "$(BOARD_BOOTLOADER_EFUSES)" ""
	$(AVRDUDE) $(AVRDUDEFLAGS) -U efuse:w:$(BOARD_BOOTLOADER_EFUSES):m
endif
ifneq "$(BOOTLOADERHEX)" ""
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(BOOTLOADERHEX):i
endif
	$(AVRDUDE) $(AVRDUDEFLAGS) -U lock:w:$(BOARD_BOOTLOADER_LOCK):m

# building the target

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

.INTERMEDIATE: $(TARGET).elf

$(TARGET).elf: $(ARDUINOLIB) $(OBJECTS)
	$(CC) $(LINKFLAGS) $(OBJECTS) $(ARDUINOLIB) -lm -o $@

%.o: %.c
	mkdir -p .dep/$(dir $<)
	$(COMPILE.c) $(CPPDEPFLAGS) -o $@ $<

%.o: %.cpp
	mkdir -p .dep/$(dir $<)
	$(COMPILE.cpp) $(CPPDEPFLAGS) -o $@ $<

%.o: %.cc
	mkdir -p .dep/$(dir $<)
	$(COMPILE.cpp) $(CPPDEPFLAGS) -o $@ $<

%.o: %.C
	mkdir -p .dep/$(dir $<)
	$(COMPILE.cpp) $(CPPDEPFLAGS) -o $@ $<

%.o: %.ino
	mkdir -p .dep/$(dir $<)
	$(COMPILE.cpp) $(CPPDEPFLAGS) -o $@ $(CPPINOFLAGS) $<

%.o: %.pde
	mkdir -p .dep/$(dir $<)
	$(COMPILE.cpp) $(CPPDEPFLAGS) -o $@ $(CPPINOFLAGS) $<

# building the arduino library

$(ARDUINOLIB): $(ARDUINOLIBOBJS)
	$(AR) rcs $@ $?

.lib/%.c.o: %.c
	mkdir -p $(dir $@)
	$(COMPILE.c) -o $@ $<

.lib/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

.lib/%.cc.o: %.cc
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

.lib/%.C.o: %.C
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

# Local Variables:
# mode: makefile
# tab-width: 4
# End: