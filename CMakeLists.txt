cmake_minimum_required(VERSION 3.6)
project(cs424mp2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    config.h
    irobot-create-ctl.cc
    irobot-create-fwd.hh
    irobot-create.cc
    irobot-create.hh
    robotest.cc)

add_executable(cs424mp2 ${SOURCE_FILES})