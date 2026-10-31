# Toolchain: arm-none-eabi-g++, qemu-system-arm
CXX      := arm-none-eabi-g++
SIZE     := arm-none-eabi-size
QEMU     := qemu-system-arm
BUILD    := build
TARGET   := $(BUILD)/firmware.elf

ARCH     := -mcpu=cortex-m3 -mthumb
CXXFLAGS := $(ARCH) -std=c++17 -Os -g -Wall -Wextra -Werror \
            -ffunction-sections -fdata-sections -ffreestanding \
            -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
            $(EXTRA_FLAGS)
LDFLAGS  := $(ARCH) -nostartfiles --specs=nano.specs -T linker.ld \
            -Wl,--gc-sections -Wl,-Map=$(BUILD)/firmware.map

SRCS := $(wildcard src/*.cpp)
OBJS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(SRCS))

QEMU_ARGS := -M mps2-an385 -cpu cortex-m3 -nographic -semihosting-config enable=on,target=native -kernel $(TARGET)

all: $(TARGET)
	@$(SIZE) $(TARGET)

$(BUILD)/%.o: src/%.cpp $(wildcard src/*.hpp) | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJS) linker.ld
	$(CXX) $(LDFLAGS) $(OBJS) -lgcc -o $@

$(BUILD):
	mkdir -p $(BUILD)

run: all
	$(QEMU) $(QEMU_ARGS)

# Fails unless the firmware reports RESULT: PASS.
# stdin is redirected because `timeout` puts QEMU in a background process group, and QEMU
# freezes if it tries to touch an interactive terminal from there.
test: all
	timeout 30 $(QEMU) $(QEMU_ARGS) < /dev/null | tee $(BUILD)/test.log
	@grep -q "RESULT: PASS" $(BUILD)/test.log && echo "TEST PASSED"

# Triggers a deliberate bus fault to exercise the HardFault handler
fault-test:
	$(MAKE) clean
	$(MAKE) all EXTRA_FLAGS=-DFAULT_TEST
	-timeout 30 $(QEMU) $(QEMU_ARGS) < /dev/null
	$(MAKE) clean

clean:
	rm -rf $(BUILD)

.PHONY: all run test fault-test clean
