#What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

#Name of this package.
project(agitr)

#Find the caktin build system, and any other packages on which we depend.
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

#Declare out catkin package.
catkin_package()

#Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files. Of there are multiple executables, use multiple copies of this line.
add_executable(hello src/hello.cpp)

# Specify libraries against which ti link. Again, this line should be copied for each distinct exectable in the package.
target_link_libraries(hello   ${catkin_LIBRARIES})


