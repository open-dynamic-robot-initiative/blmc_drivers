#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(blmc_drivers)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Dependencies
#

# Depend on ament macros.
find_package(ament_cmake REQUIRED)

# Usual dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(time_series REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rt REQUIRED)
find_package(Threads REQUIRED)
# Check for xenomai as these drivers are xenomai compatible.
find_package(Xenomai QUIET)

# Export de dependencies.
ament_export_dependencies(real_time_tools time_series Eigen3)

# Prepare the final export.
set(all_targets)
set(all_target_exports)


if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
endif()

#
# Manage the creation of the library.
#

set(blmc_drivers_src
    src/analog_sensors.cpp
    src/blmc_joint_module.cpp
    src/can_bus.cpp
    src/motor_board.cpp
    src/motor.cpp
    src/utils/polynome.cpp
)

# Create the library.
add_library(blmc_drivers SHARED ${blmc_drivers_src})

# Add the include dependencies.
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

if(Xenomai_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_INCLUDE_DIR})
endif()

# Link the catkin dependencies.
ament_target_dependencies(blmc_drivers rt)
target_link_libraries(blmc_drivers real_time_tools::real_time_tools)
target_link_libraries(blmc_drivers time_series::time_series)
target_link_libraries(blmc_drivers Eigen3::Eigen)
target_link_libraries(blmc_drivers Threads::Threads)

# If on xenomai we need to link to the real time os librairies.
if(Xenomai_FOUND)
  target_link_libraries(blmc_drivers ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()


# For the installation
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})

#
# Manage exectuables
#

add_executable(can_encoder_index_test
    src/programs/can_encoder_index_test.cpp
)
target_link_libraries(can_encoder_index_test
    ${PROJECT_NAME}
)
list(APPEND all_targets can_encoder_index_test)


#
# Manage the demos.
#

macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    demos/pd_control.cpp demos/sine_torque_control.cpp
    demos/sine_position_control.cpp demos/const_torque_control.cpp
    demos/${demo_name}.cpp)
  target_include_directories(
    ${demo_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${demo_name} blmc_drivers)
  list(APPEND all_targets ${demo_name})
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
add_demo(demo_print_analog_sensors)

#
# Install the package.
#

# Install the script to initialize the can bus
install(
  FILES ${PROJECT_SOURCE_DIR}/scripts/initialize_can_bus.sh
  RENAME initialize_can_bus
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)


#
# Tests
#
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_polynome
      tests/test_polynome.cpp
    )
    target_include_directories(test_polynome PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(test_polynome ${PROJECT_NAME})

endif()


#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package.
#
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
