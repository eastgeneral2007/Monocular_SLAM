# Debug:   cmake ..; make
# Release: cmake .. -DDO_SUPERBUILD=OFF; make

cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Superbuild retrieves and builds dependencies before building thDis project
# thus making the build process easy and eliminating the need to install build dependencies.
option(DO_SUPERBUILD "Whether a superbuild should be done" ON)

set(SOURCE_FILES main.cpp DataManager.h Frame.h AppConfig.cpp AppConfig.h KeyFrame.h KeyFrame.cpp MapPoint.h MapPoint.cpp Util.h Util.cpp ProcessingEngine.h Pipeline.h FrameLoader.cpp FrameLoader.h Tracker.cpp Tracker.h LocalMapper.cpp LocalMapper.h LoopCloser.cpp LoopCloser.h ORBFeatureExtractor.h ORBFeatureExtractor.cpp)

add_executable(Monocular_SLAM ${SOURCE_FILES})
target_link_libraries(Monocular_SLAM ${OpenCV_LIBS})