CC      = gcc
CFLAGS  = -Wall $(shell libusb-config --cflags)
LDFLAGS = $(shell libusb-config --libs) -lusb-1.0

TARGET  = kbdconfig

SRC_C   = $(TARGET).c

OBJECTS = $(SRC_C:.c=.o)
DEPS    = $(SRC_C:.c=.d)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) -lm

clean:
	rm -f $(OBJECTS) $(TARGET)

-include $(DEPS)
