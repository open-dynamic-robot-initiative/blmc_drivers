cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME blmc_can)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Xenomai REQUIRED)
add_definitions(${XENOMAI_DEFINITIONS})
message("add definitions:  " ${XENOMAI_DEFINITIONS})

# local defines
set(src_directory ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(include_directory ${CMAKE_CURRENT_SOURCE_DIR}/include)

# chaining these includes to the start
#add_definitions(-DRTSYS_XENOMAI)
include_directories(BEFORE ${include_directory} ${XENOMAI_INCLUDE_DIR})
#add_definitions(-DHAVE_INLINE)

#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags OUTPUT_VARIABLE XENO_NATIVE_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#
#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=rtdm --cflags OUTPUT_VARIABLE XENO_RTDM_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=rtdm --ldflags OUTPUT_VARIABLE XENO_RTDM_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_NATIVE_CFLAGS} ${XENO_RTDM_CFLAGS}")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_NATIVE_LDFLAGS} ${XENO_RTDM_LDFLAGS}")

#add_definitions(-O0 -Wall ${XENO_NATIVE_CFLAGS} ${XENO_RTDM_CFLAGS})

add_executable(simple_test src/simple_test.c)
target_link_libraries(simple_test
	${XENOMAI_LIBRARY_XENOMAI}
	${XENOMAI_LIBRARY_PTHREAD}
	${XENOMAI_LIBRARY_NATIVE}
	${XENOMAI_LIBRARY_RTDM}
	rt
	)

# print all cmake variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
