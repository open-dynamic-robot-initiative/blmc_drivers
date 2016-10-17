cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME blmc_can)

include(../sl_cmake_global/LAB.cmake)

include_directories(./include)


add_library(${PROJECT_NAME}
    src/can.c
    src/blmc_can.c
    src/optoforce_can.c
    )
target_link_libraries(${PROJECT_NAME} ${LAB_STD_LIBS} rtdm)


add_executable(blmc_can_demo
    src/blmc_can_demo.c
    src/can.c
    src/blmc_can.c
    src/optoforce_can.c
    )
target_link_libraries(blmc_can_demo ${PROJECT_NAME})
