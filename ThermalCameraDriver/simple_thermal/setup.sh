#!/bin/bash
# setup.sh - Setup script for thermal camera

# Check for required files
echo "Checking thermal SDK files..."

if [ ! -d "../libs" ]; then
    echo "Error: ../libs directory not found"
    exit 1
fi

# Check for required libraries
LIBS="libiruvc.so libirtemp.so libirprocess.so libirparse.so"
for lib in $LIBS; do
    if [ ! -f "../libs/$lib" ]; then
        echo "Error: Missing $lib"
        exit 1
    fi
done

# Check for calibration files
if [ ! -f "tau_L.bin" ]; then
    echo "Warning: tau_L.bin calibration file not found"
fi

if [ ! -f "tau_H.bin" ]; then
    echo "Warning: tau_H.bin calibration file not found"
fi

# Check OpenCV
if ! pkg-config --exists opencv4; then
    echo "Warning: OpenCV 4 not found. Install with:"
    echo "  sudo apt install libopencv-dev"
fi

# Check USB permissions
if ! groups | grep -q dialout; then
    echo "Adding user to dialout group for USB access..."
    sudo usermod -a -G dialout $USER
    echo "Please log out and back in for group changes to take effect"
fi

echo "Setup complete. Build with: make"