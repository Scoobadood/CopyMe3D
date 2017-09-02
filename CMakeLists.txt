cmake_minimum_required (VERSION 2.6)

set (CMAKE_CXX_STANDARD 11)

project( CopyMe3D )
set (CopyMe3D_VERSION_MAJOR 1)
set (CopyMe3D_VERSION_MINOR 0)

# Add the binary tree to the search path for inlcude files
include_directories( "src")
include_directories( "${PROJECT_BINARY_DIR}" )

# Write targets to bin directory
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/../bin)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "src/CopyMe3DConfig.h.in"
  "CopyMe3DConfig.h"
  )

# Add the binary tree to the search path for inlcude files
file(GLOB SOURCES "src/*.cpp")

add_executable(CopyMe3D ${SOURCES})



# We need OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11
target_link_libraries(CopyMe3D ${OpenCV_LIBS})
    
# And freenect2
find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIRS})
target_link_libraries(CopyMe3D ${freenect2_LIBRARIES})

# And Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# And CUDA
find_package( CUDA REQUIRED )
include_directories(${CUDA_INCLUDE_DIRS})
# Pass options to NVCC
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -std=c++11 )
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -O3 -gencode arch=compute_52,code=sm_52)
set(CUDA_SEPARABLE_COMPILATION ON)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_VERBOSE_BUILD ON)

# Add TSDF
include_directories( ${CMAKE_SOURCE_DIR}/../libsdf/include)
target_link_libraries(CopyMe3D ${CMAKE_SOURCE_DIR}/../libsdf/library/libsdf.a)


