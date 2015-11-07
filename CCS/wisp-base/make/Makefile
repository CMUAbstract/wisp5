TI_ROOT := /opt/ti
CCS_ROOT := $(TI_ROOT)/ccsv6
CG_TOOL_ROOT := $(CCS_ROOT)/tools/compiler/ti-cgt-msp430_4.4.3
CG_TOOL_LIB := $(CG_TOOL_ROOT)/bin
PLATFORM_ROOT := $(CCS_ROOT)/ccs_base/msp430
AR = $(CG_TOOL_LIB)/ar430
AS = $(CG_TOOL_LIB)/cl430
CC = $(CG_TOOL_LIB)/cl430

SRC_ROOT = ..

LIB = wisp-base.lib

COMMON_FLAGS = \
	-g \
	-vmspx \
	--abi=eabi \
	--data_model=restricted \
	--opt_for_speed=5 \
	--use_hw_mpy=F5 \
	--define=__MSP430FR5969__ \
	--printf_support=minimal \
	--silicon_errata=CPU21 \
	--silicon_errata=CPU22 \
	--silicon_errata=CPU40 \
	--advice:power="none" \
	--advice:hw_config=1 \
	--diag_warning=225 \
	--display_error_number \
	--diag_wrap=off \


CFLAGS = \
	--keep_asm \
	--preproc_with_compile \
	--include_path="$(CG_TOOL_ROOT)/include" \
	--include_path="$(PLATFORM_ROOT)/include" \
	--include_path="$(SRC_ROOT)" \

ASFLAGS = $(CFLAGS)

# Out-of-tree build: place build artifacts (object files) in the build dir
VPATH += $(SRC_ROOT)

# Specify the source files list explicitly avoiding wildcards, in order to make
# sure the build (esp. linker) doesn't pick up stray files
C_SRCS = \
	wired/spi.c \
	wired/uart.c \
	rand/rand.c \
	nvm/fram.c \
	internals/wisp-init.c \
	Timing/timer.c \
	Sensors/accel.c \
	RFID/interface.c \
	RFID/rfid_BlockWriteHandle.c \
	RFID/rfid_Handles.c \
	RFID/rfid_ReadHandle.c \
	RFID/rfid_WriteHandle.c \
	Math/crc16_LUT.c \

ASM_SRCS = \
	internals/NOPdefs.asm \
	RFID/Clocking.asm \
	RFID/RX_ISR.asm \
	RFID/Timer0A0_ISR.asm \
	RFID/Timer0A1_ISR.asm \
	RFID/TxFM0.asm \
	RFID/WISP_doRFID.asm \
	Math/crc16_ccitt.asm \

OBJS = ${C_SRCS:.c=.obj} ${ASM_SRCS:.asm=.obj}

all: $(LIB)

-include ${C_SRCS:.c=.d} ${ASM_SRCS:.asm=.d}

%.obj: %.asm
	mkdir -p "./$(shell dirname $@)"
	$(AS) $(COMMON_FLAGS) -c $(ASFLAGS) --preproc_dependency="${@:.obj=.d}" \
		--asm_directory="$(shell dirname $@)" --output_file="$@" "$<"

%.obj: %.c
	mkdir -p "./$(shell dirname $@)"
	$(CC) $(COMMON_FLAGS) -c $(CFLAGS) --preproc_dependency="${@:.obj=.d}" \
		--asm_directory="$(shell dirname $@)" --output_file="$@" "$<"

$(LIB): $(OBJS)
	$(AR) r $@ $(OBJS)

clean:
	rm -f $(LIB) $(OBJS) ${OBJS:.obj=.d} ${OBJS:.obj=.asm}
	echo $(OBJS) | xargs -n 1 dirname | sort | uniq | xargs -n 1 -I {} \
		bash -c "[ -d {} ] && find {} -empty -type d -mindepth 0 -maxdepth 0" | \
		xargs -r rmdir -p

.PHONY: clean
