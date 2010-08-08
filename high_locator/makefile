TGT=simple
#TGT=gtk
CC=gcc
DELETE=rm
ECHO=echo -e
CFLAGS+= -Ifft_001 -I./ -I/usr/include -fPIC -I/usr/include/libusb-1.0/ -std=gnu99 -Wall -DG_DISABLE_DEPRECATED -DGDK_DISABLE_DEPRECATED -DGDK_PIXBUF_DISABLE_DEPRECATED -DGTK_DISABLE_DEPRECATED `pkg-config gtk+-2.0 --cflags` -g -O0 -I../low_locator/RTOSDemo
LDFLAGS+=-L/usr/lib -lusb-1.0 `pkg-config gtk+-2.0 --libs` -s -lgtkdatabox

ifeq ($(TGT), gtk)
CFLAGS+=`pkg-config --cflags gtk+-2.0`
LDFLAGS+=`pkg-config --libs gtk+-2.0`
SOURCES=gtksteer.c
else
SOURCES=main.c \
	fft_001/radix4fft.c
#fix_fft.c 
endif
SHARED=
#SOURCES=gtksteer.c
#dpfp.c
#
OBJECTS=$(SOURCES:.c=.o)
BIN=gtksteer

BINDIR=./

all: $(BINDIR)/$(BIN)

$(BINDIR)/$(BIN): $(OBJECTS)
	$(ECHO) "----- Building oscilloscope -----"
	$(CC) $(SHARED) $(LDFLAGS) $(OBJECTS) -o $@

$(OBJECTS): %.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(DELETE) $(OBJECTS) $(BINDIR)/$(BIN)

