# Fallback build when CMake is inconvenient.
# Prefers Vendor/OpenCVHost (local source build), then Homebrew opencv.
# Experimental tests do not link OpenCV.

HOST_OPENCV ?= ../Vendor/OpenCVHost/install
OPENCV_PREFIX ?= $(shell brew --prefix opencv 2>/dev/null)
CXXFLAGS ?= -std=c++17 -O2 -I../Sphera/EngineBridge
LDFLAGS ?=
EXP_DIR := ../Sphera/EngineBridgeExperimental
EXP_CXXFLAGS ?= -std=c++17 -O2 -I$(EXP_DIR)
EXP_LDFLAGS ?= -framework Foundation -framework CoreGraphics -framework ImageIO -framework Accelerate -framework Metal

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

EXP_SRCS = \
  $(EXP_DIR)/SpheraExperimentalMath.cpp \
  $(EXP_DIR)/SpheraExperimentalImage.cpp \
  $(EXP_DIR)/SpheraExperimentalPoseOverlap.cpp \
  $(EXP_DIR)/SpheraExperimentalFeatures.cpp \
  $(EXP_DIR)/SpheraExperimentalRefinement.cpp \
  $(EXP_DIR)/SpheraExperimentalSeam.cpp \
  $(EXP_DIR)/SpheraExperimentalWarp.cpp \
  $(EXP_DIR)/SpheraExperimentalPolar.cpp \
  $(EXP_DIR)/SpheraExperimentalPipeline.cpp \
  $(EXP_DIR)/SpheraExperimentalDecode.mm \
  $(EXP_DIR)/SpheraExperimentalMetal.mm

EXP_BRIDGE_SRCS = \
  $(EXP_SRCS) \
  $(EXP_DIR)/SpheraExperimentalEngine.mm

.PHONY: all test test-exp clean

all: test_sensor_first run_native_stitch test_experimental run_experimental_metal

test_sensor_first: test_sensor_first.cpp $(BRIDGE_SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

run_native_stitch: run_native_stitch.cpp $(BRIDGE_SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Metal spherical remap is on for Mac (`!TARGET_OS_SIMULATOR`), same as iPhone.
test_experimental: test_experimental.cpp $(EXP_SRCS)
	$(CXX) $(EXP_CXXFLAGS) -fobjc-arc $^ -o $@ $(EXP_LDFLAGS)

# Same ObjC bridge the iOS app calls from ExperimentalSpheraEngine.swift.
run_experimental_metal: RunExperimentalMetalStitch.swift $(EXP_BRIDGE_SRCS)
	@rm -rf .build-exp-metal && mkdir -p .build-exp-metal
	$(CXX) $(EXP_CXXFLAGS) -fobjc-arc -c $(EXP_BRIDGE_SRCS)
	mv -f SpheraExperimental*.o .build-exp-metal/
	swiftc -O -parse-as-library \
	  -import-objc-header $(EXP_DIR)/SpheraExperimentalEngine.h \
	  -Xcc -I$(EXP_DIR) \
	  RunExperimentalMetalStitch.swift \
	  .build-exp-metal/*.o \
	  -o $@ \
	  -lc++ $(EXP_LDFLAGS)

test: test_sensor_first test_experimental
	./test_sensor_first
	./test_experimental

test-exp: test_experimental
	./test_experimental

clean:
	rm -rf .build-exp-metal
	rm -f test_sensor_first run_native_stitch test_experimental run_experimental_metal run_experimental_stitch
	rm -f SpheraExperimental*.o
