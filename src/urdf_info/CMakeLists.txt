#Required CMake version
cmake_minimum_required(VERSION 2.8.3)
#Name of package
project(urdf_info)

#Find the caktin build system, and any other packages on which we depend
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
			 roscpp
			 urdf
			 kdl_parser
			)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED system thread unit_test_framework)



#Declare our catkin package
catkin_package()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SearchForTinyXML)

#Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${tinyxml_include_dirs})
include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})


#Specify locations of link files
link_directories(${tinyxml_library_dirs})
link_directories(${console_bridge_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


#Declare definitions
add_definitions(${tinyxml_cflags})

#Declare the executable (multi line for multi executables)
add_executable(hello_world src/hello.cpp)
add_executable(full_info src/full_info.cpp)

#Specify libraries against which to link (line for each executable)
target_link_libraries(hello_world ${catkin_LIBRARIES})
target_link_libraries(full_info ${catkin_LIBRARIES})

