# Fallback build when CMake is inconvenient.
# Prefers Vendor/OpenCVHost (local source build), then Homebrew opencv.

HOST_OPENCV ?= ../Vendor/OpenCVHost/install
OPENCV_PREFIX ?= $(shell brew --prefix opencv 2>/dev/null)
CXXFLAGS ?= -std=c++17 -O2 -I../Sphera/EngineBridge
LDFLAGS ?=

# Prefer Vendor/OpenCVHost; fall back to Homebrew OpenCV via pkg-config when available.
ifneq ($(wildcard $(HOST_OPENCV)/include/opencv4/opencv2/core.hpp),)
  CXXFLAGS += -I$(HOST_OPENCV)/include/opencv4
  LDFLAGS += -L$(HOST_OPENCV)/lib \
    -lopencv_imgcodecs -lopencv_photo -lopencv_stitching -lopencv_video \
    -lopencv_calib3d -lopencv_features2d -lopencv_imgproc -lopencv_flann -lopencv_core
  LDFLAGS += -Wl,-rpath,$(HOST_OPENCV)/lib
else ifneq ($(OPENCV_PREFIX),)
  export PKG_CONFIG_PATH := $(OPENCV_PREFIX)/lib/pkgconfig:$(PKG_CONFIG_PATH)
  CXXFLAGS += $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
  LDFLAGS += $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv 2>/dev/null)
else
  $(warning No desktop OpenCV found; build Vendor/OpenCVHost or set OPENCV_PREFIX)
endif

BRIDGE_SRCS = \
  ../Sphera/EngineBridge/SpheraEngineMath.cpp \
  ../Sphera/EngineBridge/SpheraPoseOverlap.cpp \
  ../Sphera/EngineBridge/SpheraRotationRefinement.cpp \
  ../Sphera/EngineBridge/SpheraAdaptiveRingSeam.cpp \
  ../Sphera/EngineBridge/SpheraDirectSphere.cpp \
  ../Sphera/EngineBridge/SpheraPanoramaEngine.cpp

.PHONY: all test clean

all: test_sensor_first run_native_stitch

test_sensor_first: test_sensor_first.cpp $(BRIDGE_SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

run_native_stitch: run_native_stitch.cpp $(BRIDGE_SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

test: test_sensor_first
	./test_sensor_first

clean:
	rm -f test_sensor_first run_native_stitch
