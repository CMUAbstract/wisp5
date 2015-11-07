TOOLCHAIN_ROOT := /opt/ti/msp430-gcc

DEVICE  = msp430fr5969
PREFIX = $(TOOLCHAIN_ROOT)/bin/msp430-elf-
CC      = $(PREFIX)gcc

SRC_DIR := ..

CFLAGS = \
	-mmcu=$(DEVICE) \
	-O1 \
	-g \
	-std=c99 \
	-pedantic \
	-Wall \
	-I $(TOOLCHAIN_ROOT)/include \
	-I $(SRC_DIR) \

VPATH = $(SRC_DIR)

BINARY = libwisp-base.a

OBJECTS = \
	wired/spi.o \
	wired/uart.o \
	rand/rand.o \
	nvm/fram.o \
	internals/wisp-init.o \
	Timing/timer.o \
	Sensors/accel.o \
	Math/crc16_LUT.o \
	RFID/interface.o \

# TODO: RFID is written in assembly that is specific to TI compiler and
# is not compilable by GCC. Hence, for now, exclude RFID in GCC build.
#	RFID/rfid_BlockWriteHandle.c \
#	RFID/rfid_Handles.c \
#	RFID/rfid_ReadHandle.c \
#	RFID/rfid_WriteHandle.c \

all: $(BINARY)

-include $(OBJECTS:.o=.d)

%.o: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) -c -MD $(CFLAGS) $< -o $@

$(BINARY): $(OBJECTS)
	$(AR) rcs $@ $^ 

clean:
	rm -f $(OBJECTS) $(OBJECTS:.o=.d) $(BINARY)
