LIB = libwispbase

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

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/libwispbase \
