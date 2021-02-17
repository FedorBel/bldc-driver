# path to STM32F103 standard peripheral library
STD_PERIPH_LIBS ?= ./STM32F10x_StdPeriph_Lib_V3.5.0

# list of source files
SOURCES  = src/main.c
SOURCES += $(STD_PERIPH_LIBS)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
SOURCES += $(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/src/misc.c
SOURCES += $(STD_PERIPH_LIBS)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/startup_stm32f10x_md.s

# name for output binary files
PROJECT ?= bldc-driver

# compiler, objcopy (should be in PATH)
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# path to st-flash (or should be specified in PATH)
ST_FLASH ?= st-flash

# specify compiler flags
CFLAGS  = -DDEBUG -g -Wall -O
CFLAGS += -T$(STD_PERIPH_LIBS)/Project/STM32F10x_StdPeriph_Template/TrueSTUDIO/STM3210B-EVAL/stm32_flash.ld
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -mthumb-interwork
CFLAGS += -mfloat-abi=soft -mfpu=fpv4-sp-d16
CFLAGS += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
CFLAGS += -Wl,--gc-sections --specs=rdimon.specs
CFLAGS += -I.
CFLAGS += -I$(STD_PERIPH_LIBS)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
CFLAGS += -I$(STD_PERIPH_LIBS)/CMSIS/CM3/CoreSupport
CFLAGS += -I$(STD_PERIPH_LIBS)/STM32F10x_StdPeriph_Driver/inc

OBJS = $(SOURCES:.c=.o)

all: $(PROJECT).elf

# compile
$(PROJECT).elf: $(SOURCES)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin

# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin

# flash
burn:
	sudo $(ST_FLASH) write $(PROJECT).bin 0x8000000
