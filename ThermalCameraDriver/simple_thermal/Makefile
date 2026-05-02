# Makefile for Raspberry Pi 3B+ Thermal Camera
# 32-bit ARM (armv7l) with hard-float ABI

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -I../include -DLINUX -D_GNU_SOURCE -O2 -march=armv8-a -mtune=cortex-a53
LDFLAGS = -L../libs -Wl,-rpath,../libs

# OpenCV settings
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv)
OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv)

# Core libraries
CORE_LIBS = -lpthread -lusb-1.0 -lm -ldl -lrt

# Thermal SDK libraries - direct paths for arm-linux-gnueabihf
THERMAL_LIBS = ../libs/libiruvc.so ../libs/libirtemp.so ../libs/libirprocess.so ../libs/libirparse.so

# Target
TARGET = simple_thermal
SOURCE = simple_thermal.cpp

# Default target
all: check-libs $(TARGET)

# Check if libraries exist
check-libs:
	@echo "Checking thermal SDK libraries..."
	@if [ ! -f ../libs/libiruvc.so ]; then \
		echo "Error: libiruvc.so not found in ../libs/"; \
		echo "Please copy libraries from arm-linux-gnueabihf_libs/"; \
		exit 1; \
	fi
	@if [ ! -f ../libs/libirtemp.so ]; then \
		echo "Error: libirtemp.so not found in ../libs/"; \
		exit 1; \
	fi
	@if [ ! -f ../libs/libirprocess.so ]; then \
		echo "Error: libirprocess.so not found in ../libs/"; \
		exit 1; \
	fi
	@if [ ! -f ../libs/libirparse.so ]; then \
		echo "Error: libirparse.so not found in ../libs/"; \
		exit 1; \
	fi
	@echo "All libraries found"
	@echo "Architecture check:"
	@file ../libs/libiruvc.so | grep -q "32-bit.*ARM" && echo "  Correct: 32-bit ARM libraries" || echo "  Warning: Not 32-bit ARM libraries"

# Build rule
$(TARGET): $(SOURCE)
	@echo "Building thermal camera for Raspberry Pi 3B+..."
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $(TARGET) $(SOURCE) \
		$(LDFLAGS) $(CORE_LIBS) $(THERMAL_LIBS) $(OPENCV_LIBS)
	@echo "Build complete!"
	@echo "Run with: sudo ./$(TARGET)"

# Optimized build
optimized: CXXFLAGS += -O3 -ffast-math
optimized: $(TARGET)

# Debug build
debug: CXXFLAGS = -std=c++11 -I../include -DLINUX -D_GNU_SOURCE -g -O0 -DDEBUG
debug: $(TARGET)

# Check architecture
check-arch:
	@echo "System Architecture:"
	@uname -m
	@echo ""
	@echo "Expected: armv7l (32-bit ARM)"
	@echo ""
	@echo "Library Architecture:"
	@file ../libs/libiruvc.so 2>/dev/null || echo "Library not found"
	@echo ""
	@echo "ABI Type:"
	@readelf -A /bin/ls 2>/dev/null | grep Tag_ABI_VFP_args || echo "Checking..."

# Check dependencies
check-deps: $(TARGET)
	@echo "Checking runtime dependencies:"
	@ldd $(TARGET) | grep -E "(libiruvc|libirtemp|libirprocess|libirparse|libusb|opencv|not found)"

# Run target
run: $(TARGET)
	@echo "Starting thermal camera..."
	@echo "Make sure you have USB permissions (dialout group)"
	sudo ./$(TARGET)

# Install USB permissions
setup-permissions:
	@echo "Setting up USB permissions..."
	@echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5840", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-thermal-camera.rules
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger
	@sudo usermod -a -G dialout $(USER)
	@sudo usermod -a -G plugdev $(USER)
	@echo "Permissions set. Please log out and back in for group changes to take effect."

# Clean
clean:
	rm -f $(TARGET)
	@echo "Cleaned build artifacts"

# Help
help:
	@echo "Raspberry Pi 3B+ Thermal Camera Makefile"
	@echo "========================================"
	@echo ""
	@echo "Targets:"
	@echo "  make                - Build the application (default)"
	@echo "  make check-libs     - Verify thermal SDK libraries"
	@echo "  make check-arch     - Check system architecture"
	@echo "  make check-deps     - Check runtime dependencies"
	@echo "  make optimized      - Build with -O3 optimization"
	@echo "  make debug          - Build with debug symbols"
	@echo "  make run            - Build and run (requires sudo)"
	@echo "  make setup-permissions - Setup USB permissions"
	@echo "  make clean          - Remove build artifacts"
	@echo ""
	@echo "Requirements:"
	@echo "  - Raspberry Pi 3B+ running 32-bit OS (armv7l)"
	@echo "  - Libraries from arm-linux-gnueabihf_libs/"
	@echo "  - OpenCV 3.x or 4.x"
	@echo "  - libusb-1.0"

.PHONY: all check-libs check-arch check-deps run setup-permissions clean help optimized debug