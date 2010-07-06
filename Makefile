HOSTCC      = gcc
HOSTCFLAGS  = -Wall $(shell libusb-config --cflags)
HOSTLDFLAGS = $(shell libusb-config --libs) -lusb-1.0
HOSTTOOL    = kbdconfig

CC      = avr-gcc
OBJCOPY = avr-objcopy

MCU     = atmega8
FREQ	= 12000000
CFLAGS  = -I. -Wall -Os -mmcu=$(MCU) -DF_CPU=$(FREQ) -MMD
ASFLAGS = $(CFLAGS)

TARGET  = sunkbd
SRC_C   = lcd.c usbdrv/oddebug.c usbdrv/usbdrv.c sunkbd.c
SRC_ASM = usbdrv/usbdrvasm.S

OBJECTS = $(SRC_C:.c=.o) $(SRC_ASM:.S=.o)
DEPS    = $(SRC_C:.c=.d)

all: $(TARGET).hex $(HOSTTOOL)

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .data -j .text -O ihex $< $@

$(TARGET).elf: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(TARGET).map $(OBJECTS) -lm

flash: $(TARGET).hex
	avrdude -p $(MCU) -U flash:w:$(TARGET).hex

fuses:
	avrdude -p $(MCU) -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m

clean:
	rm -f $(OBJECTS) $(DEPS) *.map *.elf *.hex *.s *.i $(HOSTTOOL)

$(HOSTTOOL): $(HOSTTOOL).c usbconfig.h
	$(HOSTCC) $(HOSTCFLAGS) -o $@ $(HOSTTOOL).c $(HOSTLDFLAGS)

-include $(DEPS)
