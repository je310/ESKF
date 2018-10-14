cmake_minimum_required(VERSION 2.8.9)
project(ESKF)

include_directories(include Matrix/matrix)
file(GLOB SOURCES "src/*.cpp")

add_executable(ESKF ${SOURCES})
