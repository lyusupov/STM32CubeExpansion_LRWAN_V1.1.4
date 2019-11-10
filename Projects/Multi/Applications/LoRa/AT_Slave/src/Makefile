BASE=../../../../../..
ARM_TOOLCHAIN=/opt/gcc-arm-none-eabi/bin
GCC_PREFIX=$(ARM_TOOLCHAIN)/arm-none-eabi-
#BOARD=B_L072Z_LRWAN1
BOARD=STM32L0XX_NUCLEO
OBJ_DIR=./_build
PROJ_NAME=AT_Slave
REGION=REGION_EU868

LINKER_SCRIPT=../SW4STM32/B-L072Z-LRWAN1/mlm32l07x01/STM32L072CZYx_FLASH.ld
STARTUP_FILE=$(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/startup_stm32l072xx.s

CC=$(GCC_PREFIX)gcc
LD=$(GCC_PREFIX)ld

INC = $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/inc \
	$(BASE)/Middlewares/Third_Party/Lora/Utilities \
	$(BASE)/Middlewares/Third_Party/Lora/Phy \
	$(BASE)/Middlewares/Third_Party/Lora/Core \
	$(BASE)/Middlewares/Third_Party/Lora/Crypto \
	$(BASE)/Middlewares/Third_Party/Lora/Mac \
	$(BASE)/Middlewares/Third_Party/Lora/Mac/region \
	$(BASE)/Drivers/STM32L0xx_HAL_Driver/Inc \
	$(BASE)/Drivers/CMSIS/Include \
	$(BASE)/Drivers/CMSIS/Device/ST/STM32L0xx/Include \
	$(BASE)/Drivers/BSP/STM32L0xx_Nucleo \
	$(BASE)/Drivers/BSP/Components/sx1276 \
	$(BASE)/Drivers/BSP/Components/flash_eraseprogram \
	$(BASE)/Drivers/BSP/Components/filter \
	$(BASE)/Drivers/BSP/Components/gps \
	$(BASE)/Drivers/BSP/Components/IIC \
	$(BASE)/Drivers/BSP/Components/mpu9250 \
	$(BASE)/Drivers/BSP/Components/usart \
	$(BASE)/Drivers/BSP/MLM32L07X01 \

SRC_FILES =  $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/system_stm32l0xx.c \
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
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_spi.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lpuart.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dma.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rcc.c \
	 $(BASE)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rtc.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/low_power_manager.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/timeServer.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/delay.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/utilities.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/queue.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Utilities/trace.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Core/lora-test.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Crypto/aes.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Crypto/cmac.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/LoRaMac.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/LoRaMacCrypto.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/Region.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionAS923.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionAU915.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionCN470.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionCN779.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionCommon.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionEU433.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionEU868.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionIN865.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionKR920.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionUS915-Hybrid.c \
	 $(BASE)/Middlewares/Third_Party/Lora/Mac/region/RegionUS915.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/bsp.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/debug.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/hw_spi.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/hw_rtc.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/hw_gpio.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/stm32l0xx_it.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/stm32l0xx_hw.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/stm32l0xx_hal_msp.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/vcom.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/at.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/command.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/test_rf.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/tiny_sscanf.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/tiny_vsnprintf.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/Exti_wakeup.c \
	 $(BASE)/Drivers/BSP/Components/sx1276/sx1276.c \
	 $(BASE)/Drivers/BSP/Components/filter/filter.c \
	 $(BASE)/Drivers/BSP/Components/flash_eraseprogram/flash_eraseprogram.c \
	 $(BASE)/Drivers/BSP/Components/gps/gps.c \
	 $(BASE)/Drivers/BSP/Components/IIC/IIC.c \
	 $(BASE)/Drivers/BSP/Components/mpu9250/mpu9250.c \
	 $(BASE)/Drivers/BSP/Components/usart/bsp_usart2.c \
	 $(BASE)/Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.c \
	 $(BASE)/Drivers/BSP/sx1276mb1las/sx1276mb1las.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/main.c \
	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/lora.c

#	 $(BASE)/Drivers/BSP/B-L072Z-LRWAN1/b-l072z-lrwan1.c \
#	 $(BASE)/Drivers/BSP/MLM32L07X01/mlm32l07x01.c \
#	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/mlm32l0xx_it.c \
#	 $(BASE)/Projects/Multi/Applications/LoRa/AT_Slave/src/mlm32l0xx_hw.c \
#	 $(BASE)/Drivers/BSP/B-L072Z-LRWAN1 \
#	 $(BASE)/Middlewares/Third_Party/Lora/Core/lora.c \

TO_COMPILE = $(foreach d, $(SRC_FILES), $d)
#$(info $(TO_COMPILE))
OBJ_FILES = $(foreach d, $(SRC_FILES), $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $d)))

INC_PARAMS=$(foreach d, $(INC), -I$d)
LIB_FILES += -lc -lnosys -lm

CFLAGS += $(INC_PARAMS) -DDEBUG -DUSE_BAND_921 -DUSE_MODEM_LORA -DUSE_$(BOARD) -D$(REGION) -DSTM32L072xx -DUSE_FULL_LL_DRIVER -mthumb -mfloat-abi=soft -mcpu=cortex-m0plus -Werror
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
	$(foreach src_file, $(SRC_FILES), $(shell $(call compile_src_files, $(src_file))))
	$(CC) $(OBJ_FILES) $(STARTUP_FILE) $(LDFLAGS) $(LIB_FILES) -g -o $@

default: $(PROJ_NAME).elf

.PHONY: clean
clean:
	rm $(OBJ_DIR)/*
	rm $(PROJ_NAME).elf $(PROJ_NAME).map

.PHONY: run_openocd
run_openocd:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg -f /usr/share/openocd/scripts/target/stm32l0.cfg
