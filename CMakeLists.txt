cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME blmc_can)

include(../sl_cmake_global/LAB.cmake)

include_directories(./include)

add_executable(simple_test
    src/simple_test.c
    src/can.c
    src/blmc_can.c
    src/optoforce_can.c
    )
target_link_libraries(simple_test ${LAB_STD_LIBS} rtdm)
