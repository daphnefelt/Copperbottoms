# Makefile for FIFO RealSense Line Detection Test
# Requires: librealsense2, OpenCV

CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall `pkg-config --cflags opencv4`
LDFLAGS = -lrealsense2 `pkg-config --libs opencv4` -lpthread

TARGET_REALSENSE = fifo_realsense_linedetect
TARGET_ORIGINAL = fifo_hough_test

all: $(TARGET_REALSENSE)

$(TARGET_REALSENSE): fifo_realsense_linedetect.c
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

$(TARGET_ORIGINAL): FIFO_Hough.c
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET_REALSENSE) $(TARGET_ORIGINAL)
	rm -f fifo_*.csv

test: $(TARGET_REALSENSE)
	@echo "======================================="
	@echo "Running FIFO RealSense Line Detection Test"
	@echo "======================================="
	@echo "NOTE: Requires sudo for SCHED_FIFO scheduler"
	@echo ""
	sudo ./$(TARGET_REALSENSE)

.PHONY: all clean test
