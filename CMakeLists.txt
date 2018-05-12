cmake_minimum_required(VERSION 2.8)
project(blmc_can)

find_package(catkin REQUIRED)
find_package(Xenomai QUIET)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Use the `uname` command to determine if running on a RT_PREEMPTED system.
execute_process(COMMAND uname -v OUTPUT_VARIABLE uname_OUTPUT)

# Do nothing if Xenomai and RT_PREEMPT is not found
if(Xenomai_FOUND)
    add_definitions(${Xenomai_DEFINITIONS})

    include_directories(
        ./include
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

    add_executable(blmc_can_latency_demo
        demo/blmc_can_latency_demo.c
        src/can.c
        src/blmc_can.c
        src/optoforce_can.c
    )
    target_link_libraries(blmc_can_latency_demo ${PROJECT_NAME})

    # Demo executable
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

  # Demo executable
#  add_executable(blmc_can_demo_optoforce_dumper
#      demo/blmc_can_demo_optoforce_dumper.c
#      src/can.c
#      src/blmc_can.c
#      src/optoforce_can.c
#      )
#  target_link_libraries(blmc_can_demo_optoforce_dumper ${PROJECT_NAME})
endif()
