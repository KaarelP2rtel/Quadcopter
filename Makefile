# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf $(1)/*"
	FLASH = '$(SHELL)' -c "st-flash write build/*.bin 0x8000000"
endif

OBJDIR := build
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
	
	
install:
	$(call FLASH)	
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))


	

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# Boiler-plate
###############################################################################
# Project settings

PROJECT := Quadcopter


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += main.o
OBJECTS += nRF24L01P/nRF24L01P.o

 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/PeripheralPins.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/analogin_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/analogin_device.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/analogout_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/analogout_device.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/can_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/flash_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/gpio_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/gpio_irq_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/gpio_irq_device.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/hal_tick_overrides.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/i2c_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/lp_ticker.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_board.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_crc_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_overrides.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_retarget.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_sdk_boot.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/mbed_tz_context.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/pinmap.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/port_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/pwmout_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/pwmout_device.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/rtc_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/serial_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/serial_device.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/sleep.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/spi_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/startup_stm32f303x8.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_adc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_adc_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_can.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_cec.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_comp.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_cortex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_crc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_crc_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_dac.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_dac_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_dma.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_flash.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_flash_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_gpio.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_hrtim.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_i2c.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_i2c_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_i2s.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_i2s_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_irda.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_iwdg.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_nand.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_nor.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_opamp.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_opamp_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_pccard.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_pcd.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_pcd_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_pwr.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_pwr_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_rcc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_rcc_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_rtc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_rtc_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_sdadc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_smartcard.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_smartcard_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_smbus.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_spi.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_spi_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_sram.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_tim.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_tim_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_tsc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_uart.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_uart_ex.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_usart.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_hal_wwdg.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_adc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_comp.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_crc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_dac.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_dma.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_exti.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_fmc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_gpio.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_hrtim.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_i2c.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_opamp.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_pwr.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_rcc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_rtc.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_spi.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_tim.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_usart.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm32f3xx_ll_utils.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/stm_spi_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/system_clock.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/system_stm32f3xx.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/trng_api.o
 SYS_OBJECTS += mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/us_ticker.o

INCLUDE_PATHS += -I../.
INCLUDE_PATHS += -I../MPU6050IMU
INCLUDE_PATHS += -I../mbed
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8/TARGET_NUCLEO_F303K8
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8/device
INCLUDE_PATHS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/device
INCLUDE_PATHS += -I../mbed/drivers
INCLUDE_PATHS += -I../mbed/hal
INCLUDE_PATHS += -I../mbed/platform
INCLUDE_PATHS += -I../nRF24L01P

LIBRARY_PATHS := -L../mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM 
LIBRARIES := -lmbed 
LINKER_SCRIPT ?= ../mbed/TARGET_NUCLEO_F303K8/TOOLCHAIN_GCC_ARM/STM32F303X8.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = arm-none-eabi-gcc
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
ELF2BIN = arm-none-eabi-objcopy
PREPROC = arm-none-eabi-cpp -E -P -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp


C_FLAGS += -std=gnu99
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -DDEVICE_CRC=1
C_FLAGS += -D__MBED__=1
C_FLAGS += -DDEVICE_I2CSLAVE=1
C_FLAGS += -D__FPU_PRESENT=1
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -DTARGET_RTOS_M4_M7
C_FLAGS += -DDEVICE_RTC=1
C_FLAGS += -DTARGET_STM32F303K8
C_FLAGS += -DTARGET_NUCLEO_F303K8
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DDEVICE_CAN=1
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -DDEVICE_I2C_ASYNCH=1
C_FLAGS += -DTARGET_LIKE_CORTEX_M4
C_FLAGS += -DDEVICE_ANALOGOUT=1
C_FLAGS += -DTARGET_M4
C_FLAGS += -DTARGET_STM32F303x8
C_FLAGS += -DDEVICE_LPTICKER=1
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -DDEVICE_SPI_ASYNCH=1
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DTARGET_CORTEX
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
C_FLAGS += -D__CORTEX_M4
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1538459184.05
C_FLAGS += -DTARGET_FAMILY_STM32
C_FLAGS += -DTARGET_FF_ARDUINO
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -DTARGET_RELEASE
C_FLAGS += -DTARGET_STM
C_FLAGS += -DDEVICE_SERIAL_FC=1
C_FLAGS += -DDEVICE_USTICKER=1
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DTARGET_STM32F3
C_FLAGS += -DDEVICE_SLEEP=1
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -DDEVICE_SPISLAVE=1
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -DARM_MATH_CM4
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -std=gnu99
C_FLAGS += -c
C_FLAGS += -Wall
C_FLAGS += -Wextra
C_FLAGS += -Wno-unused-parameter
C_FLAGS += -Wno-missing-field-initializers
C_FLAGS += -fmessage-length=0
C_FLAGS += -fno-exceptions
C_FLAGS += -fno-builtin
C_FLAGS += -ffunction-sections
C_FLAGS += -fdata-sections
C_FLAGS += -funsigned-char
C_FLAGS += -MMD
C_FLAGS += -fno-delete-null-pointer-checks
C_FLAGS += -fomit-frame-pointer
C_FLAGS += -Os
C_FLAGS += -g1
C_FLAGS += -DMBED_RTOS_SINGLE_THREAD
C_FLAGS += -mcpu=cortex-m4
C_FLAGS += -mthumb
C_FLAGS += -mfpu=fpv4-sp-d16
C_FLAGS += -mfloat-abi=softfp

CXX_FLAGS += -std=gnu++98
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -DDEVICE_CRC=1
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DDEVICE_I2CSLAVE=1
CXX_FLAGS += -D__FPU_PRESENT=1
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -DTARGET_RTOS_M4_M7
CXX_FLAGS += -DDEVICE_RTC=1
CXX_FLAGS += -DTARGET_STM32F303K8
CXX_FLAGS += -DTARGET_NUCLEO_F303K8
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DDEVICE_CAN=1
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -DDEVICE_I2C_ASYNCH=1
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M4
CXX_FLAGS += -DDEVICE_ANALOGOUT=1
CXX_FLAGS += -DTARGET_M4
CXX_FLAGS += -DTARGET_STM32F303x8
CXX_FLAGS += -DDEVICE_LPTICKER=1
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -DDEVICE_SPI_ASYNCH=1
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DTARGET_CORTEX
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
CXX_FLAGS += -D__CORTEX_M4
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1538459184.05
CXX_FLAGS += -DTARGET_FAMILY_STM32
CXX_FLAGS += -DTARGET_FF_ARDUINO
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -DTARGET_RELEASE
CXX_FLAGS += -DTARGET_STM
CXX_FLAGS += -DDEVICE_SERIAL_FC=1
CXX_FLAGS += -DDEVICE_USTICKER=1
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DTARGET_STM32F3
CXX_FLAGS += -DDEVICE_SLEEP=1
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -DDEVICE_SPISLAVE=1
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -DARM_MATH_CM4
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -std=gnu++98
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -c
CXX_FLAGS += -Wall
CXX_FLAGS += -Wextra
CXX_FLAGS += -Wno-unused-parameter
CXX_FLAGS += -Wno-missing-field-initializers
CXX_FLAGS += -fmessage-length=0
CXX_FLAGS += -fno-exceptions
CXX_FLAGS += -fno-builtin
CXX_FLAGS += -ffunction-sections
CXX_FLAGS += -fdata-sections
CXX_FLAGS += -funsigned-char
CXX_FLAGS += -MMD
CXX_FLAGS += -fno-delete-null-pointer-checks
CXX_FLAGS += -fomit-frame-pointer
CXX_FLAGS += -Os
CXX_FLAGS += -g1
CXX_FLAGS += -DMBED_RTOS_SINGLE_THREAD
CXX_FLAGS += -mcpu=cortex-m4
CXX_FLAGS += -mthumb
CXX_FLAGS += -mfpu=fpv4-sp-d16
CXX_FLAGS += -mfloat-abi=softfp

ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DTRANSACTION_QUEUE_SIZE_SPI=2
ASM_FLAGS += -D__CORTEX_M4
ASM_FLAGS += -DARM_MATH_CM4
ASM_FLAGS += -D__FPU_PRESENT=1
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -I../.
ASM_FLAGS += -I../MPU6050IMU
ASM_FLAGS += -I../mbed
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8/TARGET_NUCLEO_F303K8
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/TARGET_STM32F303x8/device
ASM_FLAGS += -I../mbed/TARGET_NUCLEO_F303K8/TARGET_STM/TARGET_STM32F3/device
ASM_FLAGS += -I../mbed/drivers
ASM_FLAGS += -I../mbed/hal
ASM_FLAGS += -I../mbed/platform
ASM_FLAGS += -I../nRF24L01P
ASM_FLAGS += -include
ASM_FLAGS += /filer/workspace_data/exports/9/9c3d0a61e174112229403a49337a862e/Quadcopter/mbed_config.h
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -c
ASM_FLAGS += -Wall
ASM_FLAGS += -Wextra
ASM_FLAGS += -Wno-unused-parameter
ASM_FLAGS += -Wno-missing-field-initializers
ASM_FLAGS += -fmessage-length=0
ASM_FLAGS += -fno-exceptions
ASM_FLAGS += -fno-builtin
ASM_FLAGS += -ffunction-sections
ASM_FLAGS += -fdata-sections
ASM_FLAGS += -funsigned-char
ASM_FLAGS += -MMD
ASM_FLAGS += -fno-delete-null-pointer-checks
ASM_FLAGS += -fomit-frame-pointer
ASM_FLAGS += -Os
ASM_FLAGS += -g1
ASM_FLAGS += -DMBED_RTOS_SINGLE_THREAD
ASM_FLAGS += -mcpu=cortex-m4
ASM_FLAGS += -mthumb
ASM_FLAGS += -mfpu=fpv4-sp-d16
ASM_FLAGS += -mfloat-abi=softfp


LD_FLAGS :=-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lmbed -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  


.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@



$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(PROJECT).link_script.ld 
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter-out %.o, $^) $(LIBRARY_PATHS) --output $@ $(filter %.o, $^) $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
