cmake_minimum_required(VERSION 2.8)
project(blmc_drivers)

find_package(catkin REQUIRED)
find_package(Xenomai QUIET)
find_package(Eigen3 REQUIRED)

# support for c++ 11 -----------------------------------
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# for convenience for qtcreator, does nothing ----------
FILE(GLOB_RECURSE LibFiles *.h *.hpp)
add_custom_target(headers SOURCES ${LibFiles})

# Use the `uname` command to determine if running on a RT_PREEMPTED system.
execute_process(COMMAND uname -v OUTPUT_VARIABLE uname_OUTPUT)


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
     )

    catkin_package(
        INCLUDE_DIRS ./include
    )

    add_executable(blmc_demo_3_motors
        demos/demo_3_motors.cpp
        )
    target_link_libraries(blmc_demo_3_motors pthread)


    # unit tests ###############################################################
    catkin_add_gtest(threadsafe_object_test
      tests/main.cpp
      tests/threadsafe_object_test.cpp
    )
    target_link_libraries(threadsafe_object_test)

    catkin_add_gtest(threadsafe_timeseries_test
      tests/main.cpp
      tests/threadsafe_timeseries_test.cpp
    )
    target_link_libraries(threadsafe_timeseries_test)

endif()
