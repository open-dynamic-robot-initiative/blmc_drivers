cmake_minimum_required(VERSION 2.8)
project(blmc_drivers)

find_package(catkin REQUIRED COMPONENTS real_time_tools)
find_package(Xenomai QUIET)
find_package(Eigen3 REQUIRED)

# support for c++ 11 -----------------------------------
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Use the `uname` command to determine if running on a RT_PREEMPTED system.
execute_process(COMMAND uname -v OUTPUT_VARIABLE uname_OUTPUT)

# This macro sets the C++ preprocessor flags "XENOMAI", "RT_PREEMPT", or
# "NON_REAL_TIME" according to the current operating system.
define_os()

if(Xenomai_FOUND)
    add_definitions(${Xenomai_DEFINITIONS})

    include_directories(
        ./include
        ./new_api
        ${Xenomai_INCLUDE_DIR}
        )

    catkin_package(
      INCLUDE_DIRS ./include ${Xenomai_INCLUDE_DIR}
      LIBRARIES ${PROJECT_NAME}
      #CATKIN_DEPENDS
      DEPENDS Xenomai
    )

    # libraries ################################################################
    add_library(${PROJECT_NAME}
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(${PROJECT_NAME}
        ${Xenomai_LIBRARY_XENOMAI}
        ${Xenomai_LIBRARY_NATIVE}
        ${Xenomai_LIBRARY_RTDM}
        )

    # unit tests ###############################################################
    catkin_add_gtest(threadsafe_object_test
      tests/main.cpp
      tests/threadsafe_object_test.cpp
    )
    target_link_libraries(threadsafe_object_test ${PROJECT_NAME})

    catkin_add_gtest(threadsafe_timeseries_test
      tests/main.cpp
      tests/threadsafe_timeseries_test.cpp
    )
    target_link_libraries(threadsafe_timeseries_test ${PROJECT_NAME})


    # demos ####################################################################
    # new api ------------------------------------------------------------------
    add_executable(blmc_demo_finger
        new_api/demos/demo_finger.cpp
        )
    target_link_libraries(blmc_demo_finger ${PROJECT_NAME})

    add_executable(blmc_demo_3_motors
        new_api/demos/demo_3_motors.cpp
        )
    target_link_libraries(blmc_demo_3_motors ${PROJECT_NAME})

    add_executable(blmc_demo_single_motor
        new_api/demos/demo_single_motor.cpp
        )
    target_link_libraries(blmc_demo_single_motor ${PROJECT_NAME})

    # old api ------------------------------------------------------------------
    add_executable(blmc_can_demo
        demo/blmc_can_demo.c
        )
    target_link_libraries(blmc_can_demo ${PROJECT_NAME})

    add_executable(blmc_can_demo_optoforce_dumper
        demo/blmc_can_demo_optoforce_dumper.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(blmc_can_demo_optoforce_dumper ${PROJECT_NAME})

# elseif(${uname_OUTPUT} MATCHES "PREEMPT")
else()
    add_definitions("-D__RT_PREEMPT__")

    include_directories(
        ./include
        ${catkin_INCLUDE_DIRS}
     )

    catkin_package(
        INCLUDE_DIRS include
    )

    add_executable(demo_blmc_3_motors
        demos/demo_3_motors.cpp
        )
    target_link_libraries(demo_blmc_3_motors pthread ${catkin_LIBRARIES})

    #####
    add_executable(demo_blmc_single_board
        demos/demo_single_board.cpp
        )
    target_link_libraries(demo_blmc_single_board pthread ${catkin_LIBRARIES})

    #####
    add_executable(demo_blmc_leg
        demos/demo_leg.cpp
        )
    target_link_libraries(demo_blmc_leg pthread ${catkin_LIBRARIES})

    #####
    add_executable(demo_blmc_8_motors
        demos/demo_8_motors.cpp
        )
    target_link_libraries(demo_blmc_8_motors pthread ${catkin_LIBRARIES})

    #####
    add_executable(demo_blmc_2_motors
        demos/demo_2_motors.cpp
        )
    target_link_libraries(demo_blmc_2_motors pthread ${catkin_LIBRARIES})

    # unit tests ###############################################################
    catkin_add_gtest(test_threadsafe_object
      tests/main.cpp
      tests/threadsafe_object_test.cpp
    )
    target_link_libraries(test_threadsafe_object ${catkin_LIBRARIES})

    catkin_add_gtest(test_threadsafe_timeseries
      tests/main.cpp
      tests/threadsafe_timeseries_test.cpp
    )
    target_link_libraries(test_threadsafe_timeseries ${catkin_LIBRARIES})

endif()

catkin_install_python(PROGRAMS
  scripts/initialize_can_bus.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##########################
# building documentation #
##########################
set(BUILD_DOCUMENTATION OFF CACHE BOOL
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

