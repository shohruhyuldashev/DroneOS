# ============================================================
# DroneOS Makefile
# Real RPi Zero 2W  (Cortex-A53)
# QEMU Test Build   (Raspberry Pi 2B / Cortex-A7)
# ============================================================

# Toolchain
PREFIX   = arm-none-eabi-
CC       = $(PREFIX)gcc
AS       = $(PREFIX)as
LD       = $(PREFIX)ld
OBJCOPY  = $(PREFIX)objcopy
OBJDUMP  = $(PREFIX)objdump
SIZE     = $(PREFIX)size

# ============================================================
# Architecture Profiles
# ============================================================

ARCH_RPI  = -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard -march=armv8-a+crc
ARCH_QEMU = -mcpu=cortex-a7  -mfpu=neon-vfpv4    -mfloat-abi=hard -march=armv7-a

# Default build = Real Raspberry Pi
ARCH ?= $(ARCH_RPI)

# ============================================================
# Flags
# ============================================================

CFLAGS  = $(ARCH) -Wall -O2 -nostdlib -nostartfiles -ffreestanding -std=c11
ASFLAGS = $(ARCH)
LDFLAGS = -T linker.ld -nostdlib

# ============================================================
# Files
# ============================================================

TARGET      = droneos
KERNEL_IMG  = kernel7.img

C_SOURCES   = kernel.c
ASM_SOURCES = startup.s

OBJECTS = $(ASM_SOURCES:.s=.o) $(C_SOURCES:.c=.o)

# ============================================================
# Build Targets
# ============================================================

all: $(KERNEL_IMG)

# Real Raspberry Pi Build
rpi:
	$(MAKE) ARCH="$(ARCH_RPI)" clean all

# QEMU Test Build
qemu:
	$(MAKE) ARCH="$(ARCH_QEMU)" clean all

# Create Kernel Image
$(KERNEL_IMG): $(TARGET).elf
	$(OBJCOPY) $< -O binary $@
	@echo "====================================="
	@echo "DroneOS kernel built successfully!"
	@echo "Output: kernel7.img"
	@echo "====================================="

# Link ELF
$(TARGET).elf: $(OBJECTS) linker.ld
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@
	$(OBJDUMP) -D $@ > $(TARGET).list

# Compile C
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble ASM
%.o: %.s
	$(AS) $(ASFLAGS) $< -o $@

# ============================================================
# Utilities
# ============================================================

clean:
	rm -f *.o *.elf *.img *.list

size:
	$(SIZE) $(TARGET).elf

debug:
	$(OBJDUMP) -D $(TARGET).elf | less

# Deploy to SD card
deploy: $(KERNEL_IMG)
	@echo "Deploying to SD card..."
	@if [ -z "$(SDCARD)" ]; then \
		echo "Usage: make deploy SDCARD=/media/boot"; exit 1; \
	fi
	cp $(KERNEL_IMG) $(SDCARD)/
	cp config.txt $(SDCARD)/ 2>/dev/null || true
	sync
	@echo "Deployment complete!"

.PHONY: all clean rpi qemu deploy debug size
