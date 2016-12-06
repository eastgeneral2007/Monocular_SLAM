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
find_package(PCL 1.8 REQUIRED) # COMPONENTS common io visualization possion PassThrough NormalEstimation)
find_package(CGAL COMPONENTS core)
add_definitions(${PCL_DEFINITIONS})

# Include File Search Path
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
	    ${PCL_INCLUDE_DIRS})

# include(${CGAL_USE_FILE})

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
file(GLOB_RECURSE UNITTESTG2O_FILES "UnitTest/g2o/*.cpp" "UnitTest/g2o/*.h")
file(GLOB_RECURSE UNITTESTPCL_FILES "UnitTest/pcl/*.cpp" "UnitTest/pcl/*.h")
file(GLOB_RECURSE ERL_FILES "UnitTest/visualiseERL/*.cpp" "UnitTest/visualiseERL/*.h")
# file(GLOB_RECURSE UNITTESTCGAL_FILES "UnitTest/cgal/*.cpp" "UnitTest/cgal/*.h")
set(TESTING_SOURCES ${SOURCE_FILES})
list (REMOVE_ITEM TESTING_SOURCES "${PROJECT_SOURCE_DIR}/src/main.cpp")

# Build Targets
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${LINKED_LIBS})

# Build ERL Visualiser
add_executable(VisualiseERL ${TESTING_SOURCES} ${ERL_FILES})
target_compile_definitions(VisualiseERL PRIVATE OnlyCamera=1)
target_link_libraries(VisualiseERL ${LINKED_LIBS})

# Build UnitTestg2o
# add_executable(TestG2O ${TESTING_SOURCES} ${UNITTESTG2O_FILES})
# target_link_libraries(TestG2O ${LINKED_LIBS})

# Build UnitTestpcl
# add_executable(TestPCL ${TESTING_SOURCES} ${UNITTESTPCL_FILES})
# target_link_libraries(TestPCL ${LINKED_LIBS})

# Build UnitTestCgal
# add_executable(TestCGAL ${TESTING_SOURCES} ${UNITTESTCGAL_FILES})
# target_link_libraries(TestCGAL ${LINKED_LIBS})
