cmake_minimum_required(VERSION 2.4)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

project(interaction_flow)

#Cuda
FIND_PACKAGE(CUDA REQUIRED)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
set(CUDA_NVCC_FLAGS -arch=sm_20)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -ccbin; C:/Program Files (x86)/Microsoft Visual Studio 12.0/VC/bin)
CUDA_ADD_LIBRARY(pdflow_cudalib pdflow_cudalib.h pdflow_cudalib.cu)

#OpenCV
#FIND_PACKAGE(OpenCV REQUIRED)
set(OpenCV_LIBS )
#libFreenect2
set(FREENECT2 "/home/danfei/workspace/libfreenect2")
set(FREENECT2_lib "${FREENECT2}/build/lib/libfreenect2.so")
include_directories(${FREENECT2}/include)
link_directories(${FREENECT2}/build/lib)

set(SOURCE_FILES main.cpp kinect/kinect_interface.h kinect/kinect_interface.cpp scene_flow.cpp scene_flow.h)
add_executable(interaction_flow ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(interaction_flow ${OpenCV_LIBS} ${FREENECT2_lib} ${CUDA_LIBRARIES} pdflow_cudalib)


IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
