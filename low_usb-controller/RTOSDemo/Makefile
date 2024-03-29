#/*
#	FreeRTOS V5.4.2 - Copyright (C) 2009 Real Time Engineers Ltd.
#
#	This file is part of the FreeRTOS distribution.
#
#	FreeRTOS is free software; you can redistribute it and/or modify it	under 
#	the terms of the GNU General Public License (version 2) as published by the 
#	Free Software Foundation and modified by the FreeRTOS exception.
#	**NOTE** The exception to the GPL is included to allow you to distribute a
#	combined work that includes FreeRTOS without being obliged to provide the 
#	source code for proprietary components outside of the FreeRTOS kernel.  
#	Alternative commercial license and support terms are also available upon 
#	request.  See the licensing section of http://www.FreeRTOS.org for full 
#	license details.
#
#	FreeRTOS is distributed in the hope that it will be useful,	but WITHOUT
#	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#	FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#	more details.
#
#	You should have received a copy of the GNU General Public License along
#	with FreeRTOS; if not, write to the Free Software Foundation, Inc., 59
#	Temple Place, Suite 330, Boston, MA  02111-1307  USA.
#
#
#	***************************************************************************
#	*                                                                         *
#	* Looking for a quick start?  Then check out the FreeRTOS eBook!          *
#	* See http://www.FreeRTOS.org/Documentation for details                   *
#	*                                                                         *
#	***************************************************************************
#
#	1 tab == 4 spaces!
#
#	Please ensure to read the configuration and relevant port sections of the
#	online documentation.
#
#	http://www.FreeRTOS.org - Documentation, latest information, license and
#	contact details.
#
#	http://www.SafeRTOS.com - A version that is certified for use in safety
#	critical systems.
#
#	http://www.OpenRTOS.com - Commercial support, development, porting,
#	licensing and training services.
#*/


RTOS_SOURCE_DIR=../../low_rtos/FreeRTOS/Source
DEMO_COMMON_DIR=../../low_rtos/FreeRTOS/Demo/Common/Minimal
DEMO_INCLUDE_DIR=../../low_rtos/FreeRTOS/Demo/Common/include


CC=~/dev/arm/usr/bin/arm-elf-gcc
OBJCOPY=~/dev/arm/usr/bin/arm-elf-objcopy
SIZE=~/dev/arm/usr/bin/arm-elf-size
LDSCRIPT=atmel-rom.ld

LINKER_FLAGS=-mthumb -nostartfiles -Xlinker -oRTOSDemo.elf -Xlinker -M -Xlinker -Map=rtosdemo.map

DEBUG=
OPTIM=-O3

CFLAGS= $(DEBUG) \
		$(OPTIM) \
		-T$(LDSCRIPT) \
		-I . \
		-I./EMAC  \
		-I $(RTOS_SOURCE_DIR)/include \
		-I $(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S \
		-I $(DEMO_INCLUDE_DIR) \
		-I ./USB \
		-D SAM7_GCC \
		-D THUMB_INTERWORK \
		-mcpu=arm7tdmi \
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
		proximity_sensor/proximity_sensor.c \
		proximity_sensor/proximity_sensor_task.c \
		compass/compass_task.c \
		locator/locator_task.c \
		fft/fix_fft.c \
		adc/adc.c
#		tr24a/tr24a.c \

#		./usr/liblcd/lcd.c \
#		./usr/liblcd/lcdchar.c \
#		./usr/liblcd/lcdstr.c \
#		pio/pio.c \
#		pio/pio_it.c \

#		pwmc/pwmc.c
#		./usr/liblcd/libterm.c	\
#
#		./usr/lcdtermtask.c 
#		BasicWEB.c \
#		lwip-1.1.0/src/core/tcp_out.c \
#  		lwip-1.1.0/src/core/inet.c \
#  		lwip-1.1.0/src/core/mem.c \
#  		lwip-1.1.0/src/core/memp.c \
#		lwip-1.1.0/src/core/netif.c \
#		lwip-1.1.0/src/core/pbuf.c \
#		lwip-1.1.0/src/core/raw.c \
#		lwip-1.1.0/src/core/stats.c \
#  		lwip-1.1.0/src/core/sys.c \
#  		lwip-1.1.0/src/core/tcp.c \
#  		lwip-1.1.0/src/core/tcp_in.c \
#  		lwip-1.1.0/src/core/ipv4/ip.c \
##  		lwip-1.1.0/src/core/ipv4/ip_addr.c \
#  		lwip-1.1.0/src/core/ipv4/icmp.c \
#  		lwip-1.1.0/src/api/tcpip.c \
#  		lwip-1.1.0/src/api/api_msg.c \
#  		lwip-1.1.0/src/api/err.c \
#  		lwip-1.1.0/src/api/api_lib.c \
# 	 	lwip-1.1.0/src/netif/etharp.c \
#		EMAC/SAM7_EMAC.c \
#  		lwip-1.1.0/contrib/port/FreeRTOS/AT91SAM7X/sys_arch.c \
#  		lwip-1.1.0/src/netif/ethernetif.c \
#  		lwip-1.1.0/src/core/udp.c \
#		lwip-1.1.0/src/core/ipv4/ip_frag.c

#		./ParTest/ParTest.c \
#		$(DEMO_COMMON_DIR)/BlockQ.c \
#		$(DEMO_COMMON_DIR)/blocktim.c \
#		$(DEMO_COMMON_DIR)/flash.c \
#		$(DEMO_COMMON_DIR)/integer.c \
#		$(DEMO_COMMON_DIR)/GenQTest.c \
#		$(DEMO_COMMON_DIR)/QPeek.c \
#		$(DEMO_COMMON_DIR)/dynamic.c \
#		$(RTOS_SOURCE_DIR)/list.c \
#		$(RTOS_SOURCE_DIR)/queue.c \

#EMAC/SAM7_EMAC.c \
#		./usr/liblcd/lcd.c \
#		./usr/liblcd/lcdchar.c \
#		./usr/liblcd/lcdstr.c \
#		./usr/liblcd/libterm.c	\
#		./usr/lcdtermtask.c \


#		./webserver/uIP_Task.c \
#		./webserver/httpd.c \
#		./webserver/httpd-cgi.c \
#		./webserver/httpd-fs.c \
#		./webserver/http-strings.c \
#		./webserver/SAM7_EMAC.c \
#		$(UIP_COMMON_DIR)/uip_arp.c \
#		$(UIP_COMMON_DIR)/psock.c \
#		$(UIP_COMMON_DIR)/timer.c \
#		$(UIP_COMMON_DIR)/uip.c \


ARM_SOURCE= \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/portISR.c \
		./SrcAtmel/Cstartup_SAM7.c \
		USB/USBIsr.c \
		proximity_sensor/proximity_sensor_isr.c \
		compass/compass_isr.c

#		tr24a/spi_isr.c \

#		pwm/pwm_isr.c
#		EMAC/SAM7_EMAC_ISR.c \



#		./webserver/emac_isr.c \

THUMB_OBJS = $(THUMB_SOURCE:.c=.o)
ARM_OBJS = $(ARM_SOURCE:.c=.o)


all: RTOSDemo.bin
 
RTOSDemo.bin : RTOSDemo.hex
	$(OBJCOPY) RTOSDemo.elf -O binary RTOSDemo.bin

RTOSDemo.hex : RTOSDemo.elf
	$(OBJCOPY) RTOSDemo.elf -O ihex RTOSDemo.hex

RTOSDemo.elf : $(THUMB_OBJS) $(ARM_OBJS) boot.s Makefile
	$(CC) $(CFLAGS) $(ARM_OBJS) $(THUMB_OBJS) $(LIBS) boot.s $(LINKER_FLAGS)
	$(SIZE) RTOSDemo.elf

$(THUMB_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) -mthumb $< -o $@

$(ARM_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

program: RTOSDemo.bin
	@echo
	@echo "Programming with OPENOCD"
	./openocd_go_flash.sh

clean :
	rm $(THUMB_OBJS)
	rm $(ARM_OBJS)
	rm RTOSDemo.elf
	rm RTOSDemo.hex





