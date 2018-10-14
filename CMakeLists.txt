cmake_minimum_required(VERSION 2.8.9)
project(ESKF)

set(CMAKE_CXX_FLAGS  "-W -Wall -std=c++11")
include_directories(include Matrix/matrix)
file(GLOB SOURCES "src/*.cpp" "include/*.hpp" "Matrix/matrix/*.hpp")

add_executable(ESKF ${SOURCES})
