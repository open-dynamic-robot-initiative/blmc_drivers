######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8)

project(blmc_drivers)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
  real_time_tools
  master_board_sdk_catkin
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

# Get eigen3
search_for_eigen()

# check for xenomai as these drivers are xenomai compatible
find_package(Xenomai QUIET)

# set the catkin dependencies
set(catkin_depends
  ${CATKIN_PKGS}
)
if(Xenomai_FOUND)
  set(catkin_depends
    ${catkin_depends}
    Xenomai
  )  
endif()

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
  include_directories(
    ./include
    ./new_api
    ${Xenomai_INCLUDE_DIR}
  )
endif()


##########################################
# export the package as a catkin package #
##########################################
set(catkin_export_include_dirs include)
if(Xenomai_FOUND)
  set(catkin_export_include_dirs ${catkin_export_include_dirs} ${Xenomai_INCLUDE_DIR})
endif(Xenomai_FOUND)

catkin_package(
  INCLUDE_DIRS include ${catkin_export_include_dirs}
  LIBRARIES blmc_drivers
  CATKIN_DEPENDS ${catkin_depends}
)


########################################################
# manage the creation of the libraries and executables #
########################################################

set(blmc_drivers_src
  src/analog_sensors.cpp
  src/can_bus.cpp
  src/motor_board.cpp
  src/motor.cpp
)
# if xenomai is found we need to add the definition of the
# realtime specific functions
if(Xenomai_FOUND)
  set(blmc_drivers_src
    ${blmc_drivers_src}
    src/can.c
    src/blmc_can.c
    src/optoforce_can.c
  )
endif()

# create the library
add_library(blmc_drivers SHARED ${blmc_drivers_src})

# link the catkin dependencies
target_link_libraries(blmc_drivers ${catkin_LIBRARIES})

# if on ubuntu based OS we link to the rt and pthread packages
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(blmc_drivers  rt pthread)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# if on xenomai we need to link to the real time os librairies
if(Xenomai_FOUND)
  target_link_libraries(blmc_drivers
    ${Xenomai_LIBRARY_XENOMAI}
    ${Xenomai_LIBRARY_NATIVE}
    ${Xenomai_LIBRARY_RTDM}
  )
endif()

#########################
# manage the unit tests #
#########################



####################
# manage the demos #
####################

include_directories(${PROJECT_SOURCE_DIR}/demos)

macro(add_demo demo_name)
  add_executable(${demo_name}
    demos/pd_control.cpp
    demos/sine_torque_control.cpp
    demos/sine_position_control.cpp
    demos/const_torque_control.cpp
    demos/${demo_name}.cpp
  )
  target_link_libraries(${demo_name}
    blmc_drivers
    ${catkin_LIBRARIES}
  )
endmacro()

add_demo(demo_sine_torque_1_motor)
add_demo(demo_sine_position_1_motor)
add_demo(demo_const_torque_1_motor)
add_demo(demo_single_board)
add_demo(demo_leg)
add_demo(demo_2_motors)
add_demo(demo_1_motor_print_everything)
add_demo(demo_1_motor)
add_demo(demo_8_motors)

##########################
# building documentation #
##########################
build_doxygen_documentation()
