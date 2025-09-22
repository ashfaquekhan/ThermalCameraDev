# Makefile for simple_thermal

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -I../include -DLINUX -D_GNU_SOURCE
LDFLAGS = -L../libs -Wl,-rpath,../libs

# OpenCV settings
OPENCV_FLAGS = -I/usr/include/opencv4
OPENCV_LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

# Core libraries
CORE_LIBS = -lpthread -lusb-1.0 -lm -ldl -lrt

# Thermal SDK libraries (using .so files directly)
THERMAL_LIBS = ../libs/libiruvc.so ../libs/libirtemp.so ../libs/libirprocess.so ../libs/libirparse.so

# Target
TARGET = simple_thermal
SOURCE = simple_thermal.cpp

# Build rule
$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) -o $(TARGET) $(SOURCE) $(LDFLAGS) $(CORE_LIBS) $(THERMAL_LIBS) $(OPENCV_LIBS)

# Run target
run: $(TARGET)
	sudo ./$(TARGET)

# Clean
clean:
	rm -f $(TARGET)

.PHONY: run clean