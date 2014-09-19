
# cross compilation tools
XTOOLS_DIR = /opt/gcc-arm-none-eabi-4_8-2014q2
X_LIBC_DIR = $(XTOOLS_DIR)/arm-none-eabi/lib/armv7e-m/fpu
X_LIBGCC_DIR = $(XTOOLS_DIR)/lib/gcc/arm-none-eabi/4.8.3/armv7e-m/fpu
X_CC = $(XTOOLS_DIR)/bin/arm-none-eabi-gcc
X_OBJCOPY = $(XTOOLS_DIR)/bin/arm-none-eabi-objcopy
X_AR = $(XTOOLS_DIR)/bin/arm-none-eabi-ar
X_LD = $(XTOOLS_DIR)/bin/arm-none-eabi-ld
X_GDB = $(XTOOLS_DIR)/bin/arm-none-eabi-gdb

OUTPUT = lidar_stm32f4

# hal sources
HAL_DIR = ./hal/src
SRC += $(HAL_DIR)/stm32f4xx_hal.c \
       $(HAL_DIR)/stm32f4xx_hal_rcc.c \
       $(HAL_DIR)/stm32f4xx_hal_rcc_ex.c \
       $(HAL_DIR)/stm32f4xx_hal_pcd.c \
       $(HAL_DIR)/stm32f4xx_hal_pwr_ex.c \
       $(HAL_DIR)/stm32f4xx_hal_cortex.c \
       $(HAL_DIR)/stm32f4xx_hal_gpio.c \
       $(HAL_DIR)/stm32f4xx_hal_dma.c \
       $(HAL_DIR)/stm32f4xx_hal_dma2d.c \
       $(HAL_DIR)/stm32f4xx_hal_ltdc.c \
       $(HAL_DIR)/stm32f4xx_hal_sdram.c \
       $(HAL_DIR)/stm32f4xx_hal_i2c.c \
       $(HAL_DIR)/stm32f4xx_ll_fmc.c \
       $(HAL_DIR)/stm32f4xx_ll_usb.c \
       $(HAL_DIR)/stm32f4xx_hal_spi.c \

# bsp sources
BSP_DIR = ./bsp/STM32F429I-Discovery
SRC += $(BSP_DIR)/stm32f429i_discovery.c \
       $(BSP_DIR)/stm32f429i_discovery_lcd.c \
       $(BSP_DIR)/stm32f429i_discovery_sdram.c \
       $(BSP_DIR)/../Components/ili9341/ili9341.c \

# usb sources
USB_DIR = ./usb
SRC += $(USB_DIR)/core/usbd_core.c \
       $(USB_DIR)/core/usbd_ctlreq.c \
       $(USB_DIR)/core/usbd_ioreq.c \
       $(USB_DIR)/cdc/usbd_cdc.c \

# application sources
SRC_DIR = ./src
SRC += $(SRC_DIR)/main.c \
       $(SRC_DIR)/system_stm32f4xx.c \
       $(SRC_DIR)/stm32f4xx_it.c \
       $(SRC_DIR)/debounce.c \
       $(SRC_DIR)/gpio.c \
       $(SRC_DIR)/usbd_cdc_interface.c \
       $(SRC_DIR)/usbd_conf.c \
       $(SRC_DIR)/usbd_desc.c \
       $(SRC_DIR)/timers.c \
       $(SRC_DIR)/syscalls.c \
       $(SRC_DIR)/usart.c \
       $(SRC_DIR)/stm32f4_regs.c \
       $(SRC_DIR)/lidar.c \
       $(SRC_DIR)/pid.c \

OBJ = $(patsubst %.c, %.o, $(SRC))
OBJ += $(SRC_DIR)/start.o

# include files
INC = .
INC += ./cmsis
INC += ./fonts
INC += ./hal/inc
INC += $(USB_DIR)/core
INC += $(USB_DIR)/cdc
INC += $(BSP_DIR)
INC += $(SRC_DIR)

INCLUDE = $(addprefix -I,$(INC))

# compiler flags
CFLAGS = -Wall
CFLAGS += -O
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# linker flags
LDSCRIPT = stm32f429zi_flash.ld
LDFLAGS = -T$(LDSCRIPT) -Wl,-Map,$(OUTPUT).map -Wl,--gc-sections

DEFINES = -DSTM32F429xx

.S.o:
	$(X_CC) $(INCLUDE) $(DEFINES) $(CFLAGS) -c $< -o $@
.c.o:
	$(X_CC) $(INCLUDE) $(DEFINES) $(CFLAGS) -c $< -o $@

all: $(OBJ)
	$(X_CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -lm -o $(OUTPUT)
	$(X_OBJCOPY) -O binary $(OUTPUT) $(OUTPUT).bin

.PHONY: program
program: 
	st-flash write $(OUTPUT).bin 0x08000000

clean:
	-rm $(OBJ)	
	-rm $(OUTPUT)
	-rm $(OUTPUT).map	
	-rm $(OUTPUT).bin	
