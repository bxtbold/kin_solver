cmake_minimum_required(VERSION 3.10)
project(kin_solver)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
cmake_policy(SET CMP0167 NEW)

# Find Pinocchio package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

# Include directories
include_directories(${PINOCCHIO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/test_pinocchio.cpp
    src/interfaces/pinocchio.cpp
)

# Add executable
add_executable(test_pinocchio_kin_solver ${SOURCES})
add_definitions(-DUSE_PINOCCHIO)

# Link libraries
target_link_libraries(
    test_pinocchio_kin_solver
        ${PINOCCHIO_LIBRARIES}
        ${EIGEN3_LIBRARIES}
)
