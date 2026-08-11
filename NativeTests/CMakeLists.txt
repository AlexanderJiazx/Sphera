cmake_minimum_required(VERSION 3.16)
project(SpheraNativeTests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

execute_process(
  COMMAND brew --prefix opencv
  OUTPUT_VARIABLE OPENCV_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(OPENCV_PREFIX)
  set(OpenCV_DIR "${OPENCV_PREFIX}/lib/cmake/opencv4")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d stitching video)

set(ENGINE_BRIDGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Sphera/EngineBridge")

add_library(sphera_engine_bridge STATIC
  ${ENGINE_BRIDGE_DIR}/SpheraEngineMath.cpp
  ${ENGINE_BRIDGE_DIR}/SpheraPoseOverlap.cpp
  ${ENGINE_BRIDGE_DIR}/SpheraRotationRefinement.cpp
  ${ENGINE_BRIDGE_DIR}/SpheraAdaptiveRingSeam.cpp
  ${ENGINE_BRIDGE_DIR}/SpheraDirectSphere.cpp
  ${ENGINE_BRIDGE_DIR}/SpheraPanoramaEngine.cpp
)
target_include_directories(sphera_engine_bridge PUBLIC ${ENGINE_BRIDGE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sphera_engine_bridge PUBLIC ${OpenCV_LIBS})

add_executable(test_sensor_first test_sensor_first.cpp)
target_link_libraries(test_sensor_first PRIVATE sphera_engine_bridge)

add_executable(run_native_stitch run_native_stitch.cpp)
target_link_libraries(run_native_stitch PRIVATE sphera_engine_bridge)

enable_testing()
add_test(NAME sensor_first COMMAND test_sensor_first)
