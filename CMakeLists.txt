cmake_minimum_required (VERSION 3.8 FATAL_ERROR)

project( CopyMe3D
         VERSION 0.1 
         LANGUAGES CXX CUDA)

message( "Source directory is ${CMAKE_SOURCE_DIRECTORY}")

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



# Building the executable
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
set(CMAKE_CUDA_FLAGS "-g -G -O0")


# And libpng
find_package( PNG REQUIRED )
include_directories(${PNG_INCLUDE_DIRECTORIES})

target_compile_features(CopyMe3D PUBLIC cxx_std_11)
set_target_properties( CopyMe3D PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Add TSDF
include_directories( ${CMAKE_SOURCE_DIR}/../libsdf/include)
target_link_libraries(CopyMe3D ${CMAKE_SOURCE_DIR}/../libsdf/library/libsdf.a ${CUDA_LIBRARIES} ${PNG_LIBRARIES})


