# AVR project Makefile

# Microcontroller
MCU = atmega328p

# F_CPU (in Hz)
F_CPU = 16000000UL

# Programmer
PROGRAMMER = -c arduino -P /dev/ttyACM0 -b 115200

# Compiler
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AVRDUDE = avrdude

# Compiler flags
CFLAGS = -g -Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -fexceptions

# Source directory
SRC_DIR = src

# Build directory
BUILD_DIR = build

# Source files
SRCS = main.c $(wildcard $(SRC_DIR)/libs/**/*.c)

# Object files
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

# Output files
TARGET = main
ELF = $(TARGET).elf
HEX = $(TARGET).hex
MAP = $(TARGET).map
EEP = $(TARGET).eep

# Rules
all: $(HEX)

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $(ELF) $(HEX)

$(ELF): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(ELF)
	$(OBJDUMP) -h -S $(ELF) > $(MAP)
	$(SIZE) --format=avr --mcu=$(MCU) $(ELF)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR) $(dir $(OBJS))
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(dir $(OBJS)):
	mkdir -p $@

flash: $(HEX)
	$(AVRDUDE) $(PROGRAMMER) -p $(MCU) -U flash:w:$(HEX):i

eeprom: $(EEP)
	$(AVRDUDE) $(PROGRAMMER) -p $(MCU) -U eeprom:w:$(EEP):i

clean:
	rm -rf $(BUILD_DIR) $(ELF) $(HEX) $(MAP) $(EEP)

.PHONY: all flash eeprom clean
