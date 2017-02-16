# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)

find_package(Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

find_package(RTABMap REQUIRED)
include_directories (${RTABMap_INCLUDE_DIRS})

find_package(Rock)
rock_init(rgbd_slam 0.1)
rock_standard_layout()
