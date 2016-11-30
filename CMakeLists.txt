# Debug:   cmake ..; make
# Release: cmake .. -DDO_SUPERBUILD=OFF; make

cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Superbuild retrieves and builds dependencies before building thDis project
# thus making the build process easy and eliminating the need to install build dependencies.
option(DO_SUPERBUILD "Whether a superbuild should be done" ON)

if (DO_SUPERBUILD)
    project(SUPERBUILD NONE)
    # execute the superbuild (this script will be invoked again without the DO_SUPERBUILD option this time)
    include(cmake_modules/superbuild.cmake)
    # Stop processing this file further
    return()
else()
    project(Monocular_SLAM)
endif()

set(DEPENDENCIES_PATH ${PROJECT_SOURCE_DIR}/build/Dependencies/)
if(NOT DEFINED DEPENDENCIES_PATH)
    message(FATAL_ERROR "Variable DEPENDENCIES_PATH needs to be defined.")
endif(NOT DEFINED DEPENDENCIES_PATH)


IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Set CMake Module Path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Find Required Packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})

# Include File Search Path
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
	    ${PCL_INCLUDE_DIRS})

# Linked Library Search Path
link_directories(${PCL_LIBRARY_DIRS})

# Linked Library List
set(LINKED_LIBS ${OpenCV_LIBS} 
		${EIGEN3_LIBS} 
		${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
		${PCL_LIBRARIES})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Source Files
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp" "src/*.h")
file(GLOB_RECURSE UNITTEST_FILES "UnitTest/*.cpp" "UnitTest/*.h")
set(TESTING_SOURCES ${SOURCE_FILES})
list (REMOVE_ITEM TESTING_SOURCES "${PROJECT_SOURCE_DIR}/src/main.cpp")

# Build Targets
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${LINKED_LIBS})
add_executable(UnitTest ${TESTING_SOURCES} ${UNITTEST_FILES})
target_link_libraries(UnitTest ${LINKED_LIBS})
