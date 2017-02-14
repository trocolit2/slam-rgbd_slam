rock_library(rgbd_slam
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(rgbd_slam_bin Main.cpp
    DEPS rgbd_slam)

