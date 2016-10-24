cmake_minimum_required(VERSION 2.8)
project(blmc_can)

find_package(catkin REQUIRED)

find_package(Xenomai REQUIRED)
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
    src/blmc_can_demo.c
    src/can.c
    src/blmc_can.c
    src/optoforce_can.c
    )
target_link_libraries(blmc_can_demo ${PROJECT_NAME})
