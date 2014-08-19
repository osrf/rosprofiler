cmake_minimum_required(VERSION 2.8.3)
project(rosprofiler)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/rosprofiler scripts/rosgrapher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_profiler.test)
  add_rostest(test/test_grapher.test)
endif()
