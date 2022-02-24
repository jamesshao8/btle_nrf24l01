cmake_minimum_required(VERSION 2.8)
project(btle-tools C)

set(PACKAGE btle-tools)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake/modules)


add_definitions(-Wall)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu90")

find_package(LIBHACKRF REQUIRED)
include_directories(${LIBHACKRF_INCLUDE_DIR})
set(USE_RFBOARD "USE_HACKRF")


add_subdirectory(src)

