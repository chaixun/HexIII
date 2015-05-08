project(kinectpointcloud)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

aux_source_directory(. SRC_LIST)
#add_executable(${PROJECT_NAME} ${SRC_LIST})
add_executable(${PROJECT_NAME} main.cpp kinect.h kinect.cpp)

target_link_libraries (kinectpointcloud ${PCL_LIBRARIES} ${OpenCV_LIBS})
