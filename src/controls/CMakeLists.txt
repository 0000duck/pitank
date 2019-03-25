cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_package()

add_compile_options(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(/usr/include/)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ADD_EXECUTABLE(xbee src/xbee.cpp src/xbee_node.cpp include/xbee/xbee.hpp)
ADD_EXECUTABLE(joystick src/joystick.cpp include/joystick/joystick.hpp)
TARGET_LINK_LIBRARIES(joystick ${catkin_LIBRARIES} ${Boost_LIBRARIES} usb-1.0 -lserial)
TARGET_LINK_LIBRARIES(xbee ${catkin_LIBRARIES} -lserial)
