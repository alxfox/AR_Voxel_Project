cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(voxel_project)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(OpenCV_DIR ${LIBRARY_DIR}/install/opencv CACHE PATH "Path to installed OpenCV")

# Set C++ flags
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Define header and source files
set(HEADERS
    src/Calibration.h
    src/Model.h
    src/PoseEstimation.h
    src/Segmentation.h
    src/MarchingCubes.h
    src/VoxelCarving.h
    src/ColorReconstruction.h
    src/Postprocessing3d.h
    src/Benchmark.h
)

set(SOURCES
    src/main.cpp
    src/Model.cpp
    src/MarchingCubes.cpp
    src/VoxelCarving.cpp
    src/ColorReconstruction.cpp
    src/Postprocessing3d.cpp
)

add_executable(voxel_project ${HEADERS} ${SOURCES})
target_link_libraries(voxel_project Eigen3::Eigen ${OpenCV_LIBS})

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT voxel_project)
set_property(TARGET voxel_project PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
