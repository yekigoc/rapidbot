RTOS_SOURCE_DIR=../../low_rtos/FreeRTOS/Source
DEMO_COMMON_DIR=../../low_rtos/FreeRTOS/Demo/Common/Minimal
DEMO_INCLUDE_DIR=../../low_rtos/FreeRTOS/Demo/Common/include

CC=~/dev/arm/usr/bin/arm-elf-gcc
OBJCOPY=~/dev/arm/usr/bin/arm-elf-objcopy
SIZE=~/dev/arm/usr/bin/arm-elf-size
LDSCRIPT=atmel-rom.ld

LINKER_FLAGS=-mthumb -nostartfiles -Xlinker -orapidlocator.elf -Xlinker -M -Xlinker -Map=rtosdemo.map -lm

DEBUG=
OPTIM=-O3

CFLAGS= $(DEBUG) \
		$(OPTIM) \
		-T$(LDSCRIPT) \
		-I . \
		-I fft \
		-I./EMAC  \
		-I $(RTOS_SOURCE_DIR)/include \
		-I $(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S \
		-I $(DEMO_INCLUDE_DIR) \
		-I ./USB \
		-D SAM7_GCC \
		-D THUMB_INTERWORK \
		-mcpu=arm7tdmi-s \
		-mtune=arm7tdmi-s \
		-D AT91SAM7S256 \
		-D PACK_STRUCT_END=__attribute\(\(packed\)\) \
		-D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
		-fomit-frame-pointer \
		-mthumb-interwork

THUMB_SOURCE= \
		main.c \
		$(RTOS_SOURCE_DIR)/list.c \
		$(RTOS_SOURCE_DIR)/queue.c \
		$(RTOS_SOURCE_DIR)/tasks.c \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/port.c \
		$(RTOS_SOURCE_DIR)/portable/MemMang/heap_2.c \
		aic/aic.c \
		pwmc/pwmc.c \
		pio/pio.c \
		USB/USB-CDC.c \
		locator/locator_task.c \
		adc/adc.c \
		tc/tc.c \
		locator/locator.c
#		proximity_sensor/proximity_sensor.c \
#		proximity_sensor/proximity_sensor_task.c \
#		pwmc/pwmc.c \
#		compass/compass_task.c \
#		tr24a/tr24a.c \

ARM_SOURCE= \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/portISR.c \
		./SrcAtmel/Cstartup_SAM7.c \
		USB/USBIsr.c \
		offt/radix4fft.c \
		locator/locator_isr.c

#		fft/fix_fft.c \

THUMB_OBJS = $(THUMB_SOURCE:.c=.o)
ARM_OBJS = $(ARM_SOURCE:.c=.o)


all: rapidlocator.bin
 
rapidlocator.bin : rapidlocator.hex
	$(OBJCOPY) rapidlocator.elf -O binary rapidlocator.bin

rapidlocator.hex : rapidlocator.elf
	$(OBJCOPY) rapidlocator.elf -O ihex rapidlocator.hex

rapidlocator.elf : $(THUMB_OBJS) $(ARM_OBJS) boot.s Makefile
	$(CC) $(CFLAGS) $(ARM_OBJS) $(THUMB_OBJS) $(LIBS) boot.s $(LINKER_FLAGS)
	$(SIZE) rapidlocator.elf

$(THUMB_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) -mthumb $< -o $@

$(ARM_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

program: rapidlocator.bin
	@echo
	@echo "Programming with OPENOCD"
	./openocd_go_flash.sh

debug: rapidlocator.bin
	./openocd_go_debug.sh

clean :
	rm $(THUMB_OBJS)
	rm $(ARM_OBJS)
	rm rapidlocator.elf
	rm rapidlocator.hex
