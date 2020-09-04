cmake_minimum_required(VERSION 3.5.1)
project(Toolkit)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_BUILD_TYPE Debug)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# pcl
set(PCL_DIR $ENV{HOME}/pcl_1_8_1/share/pcl-1.8)
find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

#find_package(catkin REQUIRED COMPONENTS tf)
#include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)

# add_library(ShowUtils SHARED src/lib/ShowUtils.cpp)

# add_executable(pcap2PCD src/Pcap2PCD.cpp)
# target_link_libraries(pcap2PCD ${PCL_LIBRARIES} -lpcap)

# add_executable(mapBuilder src/MapBuilderPCD.cpp)
# target_link_libraries(mapBuilder ShowUtils ${PCL_LIBRARIES})

# add_executable(offlineEvaluate src/offlineEvaluate.cpp)
# target_link_libraries(offlineEvaluate ${PCL_LIBRARIES})

# add_executable(txt2Las src/Txt2Las.cpp)
# target_link_libraries(txt2Las ${PCL_LIBRARIES})

# add_executable(testTransform src/TestTransform.cpp)
# target_link_libraries(testTransform ${catkin_LIBRARIES})

add_executable(tq2matrix src/tq2matrix.cpp)
