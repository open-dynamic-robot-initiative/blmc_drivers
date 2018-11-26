######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8)

project(blmc_drivers)

# required to use std::shared_ptr in code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
  real_time_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

# This macro sets the C++ preprocessor flags "XENOMAI", "RT_PREEMPT", or
# "NON_REAL_TIME" according to the current operating system.
define_os()

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

########################################################
# manage the creation of the libraries and executables #
########################################################

set(blmc_drivers_src
  src/analog_sensors.cpp
  src/can_bus.cpp
  src/motor_board.cpp
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
target_link_libraries(blmc_drivers  ${catkin_LIBRARIES})

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

# here we ask to catkin to export this library in the devel folder
set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} blmc_drivers)

##########################################
# export the package as a catkin package #
##########################################

catkin_package(
  INCLUDE_DIRS include ${Xenomai_INCLUDE_DIR}
  LIBRARIES ${EXPORTED_LIBRAIRIES}
  DEPENDS ${catkin_depends}
)

#########################
# manage the unit tests #
#########################

catkin_add_gtest(test_threadsafe_object
  tests/main.cpp
  tests/threadsafe_object_test.cpp
)
target_link_libraries(test_threadsafe_object 
  blmc_drivers 
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_threadsafe_timeseries
 tests/main.cpp
 tests/threadsafe_timeseries_test.cpp
)
target_link_libraries(test_threadsafe_timeseries 
  blmc_drivers
  ${catkin_LIBRARIES}
)

####################
# manage the demos #
####################

macro(add_demo demo_name)
  add_executable(${demo_name}
    demos/${demo_name}.cpp
  )
  target_link_libraries(${demo_name}
    blmc_drivers
    ${catkin_LIBRARIES}
  )
endmacro()

add_demo(demo_single_board)
add_demo(demo_leg)
add_demo(demo_3_motors)
add_demo(demo_2_motors)
add_demo(demo_8_motors)

##########################
# building documentation #
##########################

catkin_install_python(PROGRAMS
  scripts/initialize_can_bus.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########################
# building documentation #
##########################
set(BUILD_DOCUMENTATION_${PROJECT_NAME} OFF CACHE BOOL
  "Set to ON if you want to build the documentation")
if(BUILD_DOCUMENTATION)
message(STATUS "building doxygen documentation for ${PROJECT_NAME}")
FIND_PACKAGE(Doxygen)
if (NOT DOXYGEN_FOUND)
  message(FATAL_ERROR
      "Doxygen is needed to build the documentation. "
      "Please install it correctly")
endif()
configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
# the doxygen target is generated
add_custom_target (${PROJECT_NAME}_doc ALL
  COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
  SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
endif()

###############################################
# Add custom (non compiling) targets          #
# so non cpp files show up in QT_CREATOR IDE. #
###############################################
set(USE_QTCREATOR_${PROJECT_NAME} OFF CACHE BOOL
  "Set to ON if you want QtCreator to display all files in the project tree")
if(USE_QTCREATOR_${PROJECT_NAME})
  file(GLOB_RECURSE EXTRA_FILES
      ../*.h ../*.hh ../*.hpp ../*.py
      ../*.cf  ../*.yaml ../*.txt ../*.xml
      ../start ../*.launch ../*.urdf ../CATKIN_IGNORE ../*.in
  )
  #list(FILTER EXTRA_FILES EXCLUDE REGEX "build")
  add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY
      ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
endif()

