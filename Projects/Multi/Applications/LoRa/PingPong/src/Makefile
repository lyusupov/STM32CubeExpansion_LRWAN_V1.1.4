BASE=../../../../../..
ARM_TOOLCHAIN=/opt/gcc-arm-none-eabi/bin
GCC_PREFIX=$(ARM_TOOLCHAIN)/arm-none-eabi-
BOARD=B_L072Z_LRWAN1
OBJ_DIR=./_build
PROJ_NAME=PingPong

LINKER_SCRIPT=../SW4STM32/B-L072Z-LRWAN1/mlm32l07x01/STM32L072CZYx_FLASH.ld
STARTUP_FILE=$(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/startup_stm32l072xx.s

CC=$(GCC_PREFIX)gcc
LD=$(GCC_PREFIX)ld

INC = $(BASE)/Projects/Multi/Applications/LoRa/PingPong/inc \
	$(BASE)/Middlewares/Third_Party/Lora/Utilities \
	$(BASE)/Middlewares/Third_Party/Lora/Phy \
	$(BASE)/Drivers/STM32L0xx_HAL_Driver/Inc \
	$(BASE)/Drivers/CMSIS/Include \
	$(BASE)/Drivers/CMSIS/Device/ST/STM32L0xx/Include \
	$(BASE)/Drivers/BSP/B-L072Z-LRWAN1 \
	$(BASE)/Drivers/BSP/Components/sx1276 \
	$(BASE)/Drivers/BSP/MLM32L07X01 \

SRC_FILES =  $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/system_stm32l0xx.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/low_power_manager.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/timeServer.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/delay.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/utilities.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/hw_spi.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/hw_rtc.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/hw_gpio.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/vcom.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/debug.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/mlm32l0xx_hal_msp.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/mlm32l0xx_it.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/mlm32l0xx_hw.c \
	 $(BASE)/Drivers/BSP/Components/sx1276/sx1276.c \
	 $(BASE)/Drivers/BSP/MLM32L07X01/mlm32l07x01.c \
	 $(BASE)/Drivers/BSP/B-L072Z-LRWAN1/b-l072z-lrwan1.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/PingPong/src/main.c \

TO_COMPILE = $(foreach d, $(SRC_FILES), $d)
#$(info $(TO_COMPILE))
OBJ_FILES = $(foreach d, $(SRC_FILES), $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $d)))

INC_PARAMS=$(foreach d, $(INC), -I$d)
LIB_FILES += -lc -lnosys -lm

CFLAGS += $(INC_PARAMS) -DDEBUG -DUSE_BAND_921 -DUSE_MODEM_LORA -DUSE_$(BOARD) -DSTM32L072xx -mthumb -mfloat-abi=soft -mcpu=cortex-m0plus -Werror
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums 

LDFLAGS += -mthumb -mabi=aapcs -T $(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0plus
LDFLAGS += -mfloat-abi=soft
# let linker dump unused sections
LDFLAGS += -Wl,-Map=$(PROJ_NAME).map
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs
#LDFLAGS += -z muldefs

#define compile_src_files
#
#endef

define compile_src_files
  $(CC) $(1) $(CFLAGS) $(INC_PARAMS) -MP -MD -std=c99 -g -c -o $(OBJ_DIR)/$(patsubst %.c,%.o,$(notdir $(1)))
endef

#$(OBJ_DIR)/sx1276mb1las.c.o: $(BASE)/Drivers/BSP/sx1276mb1las/sx1276mb1las.c
#	$(CC) $< $(CFLAGS) $(INC_PARAMS) -MP -MD -std=c99 -c -o $@

$(OBJ_DIR)/%.o: $(TO_COMPILE)
	$(CC) $< $(CFLAGS) $(INC_PARAMS) -g -MP -MD -std=c99 -c -o $@

$(PROJ_NAME).elf: $(SRC_FILES)
	echo "Compilation log" > compile_log.txt
	$(foreach src_file, $(SRC_FILES), $(shell $(call compile_src_files, $(src_file)) >> compile_log.txt ))
	$(CC) $(OBJ_FILES) $(STARTUP_FILE) $(LDFLAGS) $(LIB_FILES) -g -o $@

default: $(PROJ_NAME).elf

.PHONY: clean
clean:
	rm $(OBJ_DIR)/*
	rm $(PROJ_NAME).elf $(PROJ_NAME).map

.PHONY: run_openocd
run_openocd:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg -f /usr/share/openocd/scripts/target/stm32l0.cfg