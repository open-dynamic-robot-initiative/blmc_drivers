cmake_minimum_required(VERSION 2.8)
project(blmc_can)

find_package(catkin REQUIRED)
find_package(Xenomai QUIET)
find_package(Eigen3 REQUIRED)

# support for c++ 11 -----------------------------------
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# for convenience for qtcreator, does nothing ----------
FILE(GLOB_RECURSE LibFiles *.h *.hpp)
add_custom_target(headers SOURCES ${LibFiles})

# Use the `uname` command to determine if running on a RT_PREEMPTED system.
execute_process(COMMAND uname -v OUTPUT_VARIABLE uname_OUTPUT)


# Do nothing if Xenomai and RT_PREEMPT is not found
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
    add_executable(api_demo_finger
        new_api/demos/api_demo_finger.cpp
        )
    target_link_libraries(api_demo_finger ${PROJECT_NAME})

    add_executable(api_demo_3_motors
        new_api/demos/api_demo_3_motors.cpp
        )
    target_link_libraries(api_demo_3_motors ${PROJECT_NAME})

    add_executable(api_demo_single_motor
        new_api/demos/api_demo_single_motor.cpp
        )
    target_link_libraries(api_demo_single_motor ${PROJECT_NAME})

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
        ./new_api

     )

    catkin_package(
        INCLUDE_DIRS ./include
        LIBRARIES ${PROJECT_NAME}
    )


    # The CAN API library
    add_library(${PROJECT_NAME}
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
    )

    # Demo executable
    add_executable(blmc_can_demo
        demo/blmc_can_demo.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
    )
    target_link_libraries(blmc_can_demo ${PROJECT_NAME} pthread)

    add_executable(blmc_can_latency_demo
        demo/blmc_can_latency_demo.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
    )
    target_link_libraries(blmc_can_latency_demo ${PROJECT_NAME} pthread)


    add_executable(blmc_can_sine_demo
        demo/blmc_can_sine_demo.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
    )
    target_link_libraries(blmc_can_sine_demo ${PROJECT_NAME} pthread m)



    add_executable(api_demo_finger
        new_api/demos/api_demo_finger.cpp
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(api_demo_finger ${PROJECT_NAME} pthread)

    add_executable(api_demo_3_motors
        new_api/demos/api_demo_3_motors.cpp
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(api_demo_3_motors ${PROJECT_NAME} pthread)


    add_executable(api_demo_single_motor
        new_api/demos/api_demo_single_motor.cpp
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(api_demo_single_motor ${PROJECT_NAME} pthread)

  # Demo executable
#  add_executable(blmc_can_demo_optoforce_dumper
#      demo/blmc_can_demo_optoforce_dumper.c
#      src/can.c
#      src/blmc_can.c
#      src/optoforce_can.c
#      )
#  target_link_libraries(blmc_can_demo_optoforce_dumper ${PROJECT_NAME})
endif()





