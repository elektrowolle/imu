#cmake_minimum_required (VERSION 2.6)
#project (Tutorial)
##include(mbed-os/hal/CMakeLists.txt)
#include_directories(mbed-os/hal/api)
#add_subdirectory(mbed-os/hal)

set(MBED_TARGET NUCLEO_L476RG)

MACRO(FILTER_PATH file_path PATH)
    GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
ENDMACRO()

MACRO(HEADER_DIRECTORIES return_list pattern exclude)
    message(${pattern})
    FILE(GLOB_RECURSE new_list ${pattern})
    SET(dir_list "")
    FOREACH(file_path ${new_list})
#        if(NOT ${file_path} MATCHES ${exclude})
            FILTER_PATH(${file_path} PATH)
#            GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
            SET(dir_list ${dir_list} ${file_path})
#        endif()
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()


set(target mbed)
set(legacy_magic_def  "**G/*.h")
#set(MBED_LEGACY_TARGET_DEFINITIONS "${legacy_magic_def}")


#HEADER_DIRECTORIES(header_list "*.h" ".*TARGET_.*")

HEADER_DIRECTORIES(target_list ${legacy_magic_def} "")

#
foreach(header ${header_list})
    message(${header})
endforeach(header)
foreach(header ${target_list})
    message(${header})
endforeach(header)

cmake_minimum_required(VERSION 3.2.2) # CMake version check
project(simple_example)               # Create project "simple_example"
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # Enable c++11 standard


#include_directories(mbed-os/hal)

SET(target_include "")

#HEADER_DIRECTORIES(taget_includes)

set(HEADER_FILES ${header_list})
set(SOURCE_FILES main.cpp bno055/IMU.cpp ${header_list})
add_executable(simple_example ${SOURCE_FILES})
