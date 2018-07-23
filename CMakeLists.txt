cmake_minimum_required(VERSION 2.8)
project(blmc_can)

find_package(catkin REQUIRED)
find_package(Xenomai QUIET)
find_package(Eigen REQUIRED)

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


    # The CAN API library
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
    # Demo executable
    add_executable(blmc_can_demo
        demo/blmc_can_demo.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(blmc_can_demo ${PROJECT_NAME})


    add_executable(new_api_demo
        new_api/new_api_demo.cpp
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(new_api_demo ${PROJECT_NAME})


    add_executable(newest_api_demo
        new_api/newest_api_demo.cpp
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(newest_api_demo ${PROJECT_NAME})

#    add_executable(blmc_can_latency_demo
#        demo/blmc_can_latency_demo.c
#        src/can.c
#        src/blmc_can.c
#        src/optoforce_can.c
#    )
#    target_link_libraries(blmc_can_latency_demo ${PROJECT_NAME})

    # Demo executable
    add_executable(blmc_can_demo_optoforce_dumper
        demo/blmc_can_demo_optoforce_dumper.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
        )
    target_link_libraries(blmc_can_demo_optoforce_dumper ${PROJECT_NAME})



    ####################################################################################
    #                       example of unit tests                                      #
    ####################################################################################

    # to compile and run:
    # after calling catkin_make, source ./devel/setup.bash
    # then after typing "catkin_make run_", you may press "tab" for autocompletion
    # selecting "catkin_make run_tests" will results in all found tests to be compiled
    # and run, but you may select specific test

    catkin_add_gtest(blmc_can_tests
      tests/main.cpp
      tests/threadsafe_object_test.cpp
    )
    target_link_libraries(blmc_can_tests
      ${PROJECT_NAME})



# elseif(${uname_OUTPUT} MATCHES "PREEMPT")
else()
    add_definitions("-D__RT_PREEMPT__")

    include_directories(
        ./include
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

  # Demo executable
#  add_executable(blmc_can_demo_optoforce_dumper
#      demo/blmc_can_demo_optoforce_dumper.c
#      src/can.c
#      src/blmc_can.c
#      src/optoforce_can.c
#      )
#  target_link_libraries(blmc_can_demo_optoforce_dumper ${PROJECT_NAME})
endif()





