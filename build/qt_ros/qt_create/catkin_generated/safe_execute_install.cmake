execute_process(COMMAND "/home/sergio/Downloads/3pi_gaming/build/qt_ros/qt_create/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sergio/Downloads/3pi_gaming/build/qt_ros/qt_create/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
