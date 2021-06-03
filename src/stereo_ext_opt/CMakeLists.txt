cmake_minimum_required(VERSION 3.10)
project(stereo_ext_opt)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

################# dependencies #################

# catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# Thread
find_package (Threads)

set(THIRD_PARTY_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES stereo_ext_opt
    )

################### source #####################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
