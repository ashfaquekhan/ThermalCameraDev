#!/bin/bash
# Setup script for P2 Thermal Camera on Raspberry Pi 3B+
# This script checks dependencies and sets up the environment

set -e

echo "========================================"
echo "P2 Thermal Camera Setup - Raspberry Pi 3B+"
echo "========================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on correct architecture
echo "1. Checking system architecture..."
ARCH=$(uname -m)
if [ "$ARCH" = "armv7l" ]; then
    echo -e "${GREEN}✓${NC} Correct architecture: $ARCH (32-bit ARM)"
elif [ "$ARCH" = "aarch64" ]; then
    echo -e "${YELLOW}⚠${NC} Warning: Running 64-bit OS on RPi 3B+"
    echo "   You should use aarch64-linux-gnu_libs instead"
    echo "   Current setup uses arm-linux-gnueabihf_libs (32-bit)"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${RED}✗${NC} Unexpected architecture: $ARCH"
    echo "   This setup is for Raspberry Pi 3B+ (armv7l or aarch64)"
    exit 1
fi
echo ""

# Check directory structure
echo "2. Checking directory structure..."
if [ ! -d "../include" ]; then
    echo -e "${RED}✗${NC} Error: ../include directory not found"
    echo "   Please run this from the build directory"
    exit 1
fi
echo -e "${GREEN}✓${NC} Include directory found"

if [ ! -d "../libs" ]; then
    echo -e "${RED}✗${NC} Error: ../libs directory not found"
    echo "   Expected structure:"
    echo "   ThermalCam/"
    echo "   ├── include/"
    echo "   ├── libs/  (arm-linux-gnueabihf_libs)"
    echo "   └── simple_thermal/ (your code here)"
    exit 1
fi
echo -e "${GREEN}✓${NC} Libraries directory found"
echo ""

# Check for required libraries
echo "3. Checking thermal SDK libraries..."
LIBS_FOUND=true
for lib in libiruvc.so libirtemp.so libirprocess.so libirparse.so; do
    if [ -f "../libs/$lib" ]; then
        # Check if it's the correct architecture
        LIBARCH=$(file ../libs/$lib | grep -o "32-bit.*ARM" || echo "")
        if [ -n "$LIBARCH" ]; then
            echo -e "${GREEN}✓${NC} $lib (32-bit ARM)"
        else
            echo -e "${YELLOW}⚠${NC} $lib found but may be wrong architecture"
            file ../libs/$lib
        fi
    else
        echo -e "${RED}✗${NC} Missing: $lib"
        LIBS_FOUND=false
    fi
done

if [ "$LIBS_FOUND" = false ]; then
    echo ""
    echo -e "${RED}Error: Missing required libraries${NC}"
    echo "Please copy libraries from:"
    echo "  AC010_256_SDK_V2.0.2/SINGLE_USB/libs/linux/arm-linux-gnueabihf_libs/"
    echo "to:"
    echo "  ../libs/"
    exit 1
fi
echo ""

# Check for calibration files (optional but recommended)
echo "4. Checking calibration files..."
if [ -f "tau_L.bin" ] && [ -f "tau_H.bin" ]; then
    echo -e "${GREEN}✓${NC} Calibration files found (tau_L.bin, tau_H.bin)"
elif [ -f "../tau_L.bin" ] && [ -f "../tau_H.bin" ]; then
    echo -e "${YELLOW}⚠${NC} Calibration files in parent directory"
    echo "   Copying to current directory..."
    cp ../tau_L.bin .
    cp ../tau_H.bin .
    echo -e "${GREEN}✓${NC} Calibration files copied"
else
    echo -e "${YELLOW}⚠${NC} Calibration files not found"
    echo "   Temperature readings may be less accurate"
    echo "   Files should be: tau_L.bin, tau_H.bin"
fi
echo ""

# Check for system dependencies
echo "5. Checking system dependencies..."

# Check for g++
if command -v g++ &> /dev/null; then
    GCC_VERSION=$(g++ --version | head -n1)
    echo -e "${GREEN}✓${NC} g++ compiler: $GCC_VERSION"
else
    echo -e "${RED}✗${NC} g++ compiler not found"
    echo "   Install with: sudo apt install build-essential"
    exit 1
fi

# Check for pkg-config
if command -v pkg-config &> /dev/null; then
    echo -e "${GREEN}✓${NC} pkg-config found"
else
    echo -e "${RED}✗${NC} pkg-config not found"
    echo "   Install with: sudo apt install pkg-config"
    exit 1
fi

# Check for OpenCV
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    echo -e "${GREEN}✓${NC} OpenCV 4 found (version $OPENCV_VERSION)"
elif pkg-config --exists opencv; then
    OPENCV_VERSION=$(pkg-config --modversion opencv)
    echo -e "${GREEN}✓${NC} OpenCV 3 found (version $OPENCV_VERSION)"
else
    echo -e "${RED}✗${NC} OpenCV not found"
    echo "   Install with: sudo apt install libopencv-dev"
    exit 1
fi

# Check for libusb
if pkg-config --exists libusb-1.0; then
    LIBUSB_VERSION=$(pkg-config --modversion libusb-1.0)
    echo -e "${GREEN}✓${NC} libusb-1.0 found (version $LIBUSB_VERSION)"
else
    echo -e "${RED}✗${NC} libusb-1.0 not found"
    echo "   Install with: sudo apt install libusb-1.0-0-dev"
    exit 1
fi
echo ""

# Check USB permissions
echo "6. Checking USB permissions..."
GROUPS_OUTPUT=$(groups)
if echo "$GROUPS_OUTPUT" | grep -q "dialout"; then
    echo -e "${GREEN}✓${NC} User in dialout group"
else
    echo -e "${YELLOW}⚠${NC} User not in dialout group"
    echo "   Adding user to dialout group..."
    sudo usermod -a -G dialout $USER
    echo -e "${GREEN}✓${NC} Added to dialout group (requires logout to take effect)"
fi

if echo "$GROUPS_OUTPUT" | grep -q "plugdev"; then
    echo -e "${GREEN}✓${NC} User in plugdev group"
else
    echo -e "${YELLOW}⚠${NC} User not in plugdev group"
    echo "   Adding user to plugdev group..."
    sudo usermod -a -G plugdev $USER
    echo -e "${GREEN}✓${NC} Added to plugdev group (requires logout to take effect)"
fi

# Setup udev rules
echo ""
echo "7. Setting up USB udev rules..."
UDEV_RULE_FILE="/etc/udev/rules.d/99-thermal-camera.rules"
UDEV_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5840", MODE="0666", GROUP="plugdev"'

if [ -f "$UDEV_RULE_FILE" ]; then
    echo -e "${GREEN}✓${NC} USB udev rules already exist"
else
    echo "   Creating udev rule..."
    echo "$UDEV_RULE" | sudo tee "$UDEV_RULE_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}✓${NC} USB udev rules created"
fi
echo ""

# Check for thermal camera
echo "8. Checking for thermal camera..."
if lsusb | grep -q "0bda:5840"; then
    echo -e "${GREEN}✓${NC} P2 Thermal Camera detected"
    lsusb | grep "0bda:5840"
else
    echo -e "${YELLOW}⚠${NC} P2 Thermal Camera not detected"
    echo "   Make sure the camera is connected via USB"
    echo "   Camera should appear as VID=0x0BDA, PID=0x5840"
fi
echo ""

# Summary
echo "========================================"
echo "Setup Summary"
echo "========================================"
echo ""
echo "System: Raspberry Pi 3B+ ($ARCH)"
echo "Libraries: arm-linux-gnueabihf (32-bit ARM hard-float)"
echo ""

if [ "$LIBS_FOUND" = true ]; then
    echo -e "${GREEN}✓${NC} All required libraries present"
else
    echo -e "${RED}✗${NC} Some libraries missing"
fi

echo ""
echo "Next steps:"
echo "1. Build the application:"
echo "   make"
echo ""
echo "2. Run the application:"
echo "   sudo ./simple_thermal"
echo ""
echo "3. If you added to groups, log out and back in first:"
echo "   logout"
echo ""

if [ -f "Makefile" ]; then
    echo "Makefile found. You can also use:"
    echo "  make check-libs  - Verify libraries"
    echo "  make check-arch  - Check architecture"
    echo "  make run         - Build and run"
    echo ""
fi

echo "========================================"
echo "Setup complete!"
echo "========================================"