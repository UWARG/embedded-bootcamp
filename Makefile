##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [2.30.0] date: [Sat Oct 06 18:34:33 EDT 2018] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = Bootcamp


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
PROJECT_DIR 	= .

# Build path
BUILD_DIR 		= $(PROJECT_DIR)/build
HAL_DIR 		= $(PROJECT_DIR)/Drivers/STM32F0xx_HAL_Driver

######################################
# source
######################################
# C sources
HAL_SOURCES = $(wildcard $(HAL_DIR)/Src/*.c)

MAIN_SOURCES = $(wildcard $(PROJECT_DIR)/Src/*.c)

C_SOURCES = \
$(HAL_SOURCES) \
$(MAIN_SOURCES) \
Src/main.c \
Src/gpio.c \
Src/adc.c \
Src/tim.c \
Src/usart.c \
Src/stm32f0xx_it.c \
Src/stm32f0xx_hal_msp.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c \
Src/system_stm32f0xx.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f030xc.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
BINPATH = 
PREFIX = arm-none-eabi-
CC = $(BINPATH)$(PREFIX)gcc
AS = $(BINPATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)$(PREFIX)objcopy
AR = $(BINPATH)$(PREFIX)ar
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F030xC


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F0xx_HAL_Driver/Inc \
-IDrivers/STM32F0xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F0xx/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
C_DEFS += -DDEBUG
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F030RCTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo $<
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo $<
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo $@
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo $@
	@$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo $@
	@$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

# program device
install: $(BUILD_DIR)/$(TARGET).bin
	@ST-LINK_CLI -c SWD -p "$(BUILD_DIR)/$(TARGET).bin" 0x08000000 -NoPrompt -Rst -Run

# *** EOF ***
