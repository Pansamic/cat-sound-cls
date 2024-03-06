cmake_minimum_required(VERSION 3.8)
project(wav2mfcc-client)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME}
    src/list_ports_linux.cc
    src/main.cpp
    src/serial.cc
    src/unix.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
