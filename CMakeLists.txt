cmake_minimum_required(VERSION 2.8.3)
project(data_compensate_inmotion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_compile_options(-std=c++11)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "/home/hyb/jzarm/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "/home/hyb/jzarm/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-g++")

# set(CMAKE_FIND_ROOT_PATH "/home/hyb/")
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(DATA_COMPENSATE_PATH "./")
set(SOURCE_FILES
      src/Multi-SensorsDataReceive.cpp
)

include_directories(${DATA_COMPENSATE_PATH}/include  ${DATA_COMPENSATE_PATH}/src)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
# add_executable(${PROJECT_NAME} src/test_main.cpp src/Multi-SensorsDataReceive.cpp)
# target_link_libraries(${PROJECT_NAME} -pthread)


