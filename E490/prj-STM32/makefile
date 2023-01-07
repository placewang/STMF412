#OPT	= s

#BOOT = 0
#2S = 1
#ENCODER_SUPPORT = 1
STM32_BOARD = 1

ifdef BOOT
TARGET = boot
START_ADDR = 0x08000000
else
ifdef 2S
TARGET = main-2s
else
TARGET = main-1s
endif
START_ADDR = 0x08010000
endif

ifdef 2S
SYS_SUPPORT = SUPPORT_2S
else
SYS_SUPPORT = SUPPORT_1S
endif

ARCH_DIR = ../arch_STM32103
MISC_DIR = ../misc
CORE_DIR = ../core

ifdef 2S
DIR_OBJ = ./obj_2s
else
DIR_OBJ = ./obj_1s
endif

ifndef BOOT
USR_INCS+= -I $(CORE_DIR)/../SShell
endif

MISC_SRCS = $(MISC_DIR)/dprintf.c \
	    $(MISC_DIR)/syscalls.c \

ifndef BOOT
MISC_SRCS += $(MISC_DIR)/commands.c
endif

ARCH_SRCS = $(ARCH_DIR)/src/time.c \
	    $(ARCH_DIR)/src/uart.c \
	    $(ARCH_DIR)/src/gpio.c \
	    $(ARCH_DIR)/src/fsmc.c \
	    $(ARCH_DIR)/src/eeprom.c \

ifdef BOOT

ARCH_SRCS += $(ARCH_DIR)/src_boot/can.c \
	     $(ARCH_DIR)/src_boot/stm32f10x_it.c \
	     $(ARCH_DIR)/src_boot/arch_stm32.c \
	     $(ARCH_DIR)/src_boot/upgrade.c \

else

ARCH_SRCS += $(ARCH_DIR)/src/can.c \
	     $(ARCH_DIR)/src/stm32f10x_it.c \
	     $(ARCH_DIR)/src/arch_stm32.c \
	     $(ARCH_DIR)/src/exti.c \
	     $(ARCH_DIR)/src/iwdg.c \
	     $(ARCH_DIR)/src/adc.c \
	     $(ARCH_DIR)/src/coder.c \

endif

ifdef BOOT
CORE_SRCS = $(CORE_DIR)/src/boot.c
else
CORE_SRCS = $(CORE_DIR)/src/alert.c \
	    $(CORE_DIR)/src/main.c \
	    $(CORE_DIR)/src/massage.c \
	    $(CORE_DIR)/src/step.c \
	    $(CORE_DIR)/src/ver.c \
	    $(CORE_DIR)/../SShell/shell.c \

endif

SOURCES = $(CORE_SRCS) $(ARCH_SRCS) $(MISC_SRCS)

OBJECTS = $(SOURCES:../%.c=$(DIR_OBJ)/%.o)

USR_INCS+= -I $(CORE_DIR)/inc
USR_INCS+= -I $(ARCH_DIR)/inc

LIBSTM32 = ../arch_STM32103/lib/libstm32f10x.a

TCHAIN = arm-none-eabi

INCLUDE_DIRS = -I . -I $(ARCH_DIR)/lib/inc $(USR_INCS)
#COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall -g -O$(OPT)
COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall -O$(OPT)
LIBRARY_DIRS = -L ../arch_STM32103/lib $(USR_LIBS)

ARCH_CC = $(TCHAIN)-gcc
ARCH_CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS) -D$(SYS_SUPPORT)

#
# Implicit Rules
#
$(DIR_OBJ)/%.o : ../%.c
	@$(call makedir, $@)
	$(ARCH_CC) $(ARCH_CFLAGS) -c -o $@ $<

$(DIR_OBJ)/%.o : ../%.s
	@$(call makedir, $@)
	$(ARCH_CC) $(ARCH_CFLAGS) -c -o $@ $<


ifdef ENCODER_SUPPORT
ARCH_CFLAGS += -DENCODER_SUPPORT
endif
ifdef ZERO2_SUPPORT
ARCH_CFLAGS += -DZERO2_SUPPORT
endif
ifdef STM32_BOARD
ARCH_CFLAGS += -DSTM32_BOARD
endif

ARCH_LD = $(TCHAIN)-gcc
ifdef BOOT
ARCH_LDFLAGS = -Wl,--gc-sections,-Map=$(TARGET).map,-cref,-u,Reset_Handler -D__MICROLIB $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T ../arch_STM32103/stm32_flash_boot.ld
else
ARCH_LDFLAGS = -Wl,--gc-sections,-Map=$(TARGET).map,-cref,-u,Reset_Handler -D__MICROLIB $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T ../arch_STM32103/stm32_flash.ld
endif

OBJCP = $(TCHAIN)-objcopy
OBJCPFLAGS = -O binary
OBJCPFLAGSHEX = -O ihex

AR = $(TCHAIN)-ar
ARFLAGS = cr

MAIN_OUT = $(TARGET)
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin
MAIN_OUT_HEX = $(MAIN_OUT).hex

STR_DIV	= ------------------------------------------------------------

# all

#all: start gcc-info $(MAIN_OUT_ELF) $(MAIN_OUT_BIN) flash end
all: start gcc-info $(MAIN_OUT_ELF) $(MAIN_OUT_BIN) $(MAIN_OUT_HEX) end

# misc

start:
	@echo Start Compiling Target $(TARGET)

end:
	hex2pgx_V6.exe --headall \
		--exec-8b 32774:main-1s.hex \
		dummy-dummy.hex head_new1sys.pgh

	#hex2pgx_V6.exe --headall \
	#	--exec-8b 32773:main-1s.hex \
	#	dummy-dummy.hex head.pgh

	@echo $(STR_DIV)
	@echo bye!!

gcc-info:
	@echo $(STR_DIV)
	@echo gcc version is
	@$(CC) --version
	@echo $(STR_DIV)

# main

$(MAIN_OUT_ELF): $(OBJECTS) $(LIBSTM32)
	$(ARCH_LD) $(ARCH_LDFLAGS) $(OBJECTS) $(LIBSTM32) --output $@

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@

$(MAIN_OUT_HEX): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGSHEX) $< $@


# flash

flash:
	@echo $(STR_DIV)
	stm32boot -a $(START_ADDR) -r -e -v -x $(MAIN_OUT_BIN)
	minicom

lib: $(LIBSTM32)

# libstm32.a

#LIBSTM32 = lib/libstm32.a

LIBSTM32_SRCS = \
		$(ARCH_DIR)/lib/src/stm32f2xx_adc.c \
		$(ARCH_DIR)/lib/src/stm32f10x_bkp.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_can.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_dac.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_dma.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_exti.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_flash.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_gpio.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_i2c.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_iwdg.c \
		$(ARCH_DIR)/lib/src/stm32f10x_lib.c \
		$(ARCH_DIR)/lib/src/stm32f10x_nvic.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_pwr.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_rcc.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_rtc.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_spi.c \
		$(ARCH_DIR)/lib/src/stm32f10x_systick.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_tim.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_usart.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_fsmc.c \
		$(ARCH_DIR)/lib/src/stm32f2xx_wwdg.c \
		$(ARCH_DIR)/lib/startup/gcc/stm32f10x_vector.c \
		$(ARCH_DIR)/lib/startup/gcc/cortexm3_macro.s \


LIBSTM32_OBJS1 = $(LIBSTM32_SRCS:../%.c=./obj/%.o)
LIBSTM32_OBJS = $(LIBSTM32_OBJS1:../%.s=./obj/%.o)

./obj/%.o : ../%.c
	@$(call makedir, $@)
	$(ARCH_CC) $(ARCH_CFLAGS) -c -o $@ $<

./obj/%.o : ../%.s
	@$(call makedir, $@)
	$(ARCH_CC) $(ARCH_CFLAGS) -c -o $@ $<

$(LIBSTM32): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBSTM32_OBJS): ../arch_STM32103/inc/stm32f10x_conf.h

# User Code

clean:
	-rm -f $(OBJECTS) $(TARGET).o $(TARGET) $(TARGET).map $(LIBSTM32_OBJS) $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)

#
# by xhl 2012/07/15
#
makedir = \
	@if ! test -d $(dir $(1)); \
	then \
		mkdir -p $(dir $(1)); \
	fi

