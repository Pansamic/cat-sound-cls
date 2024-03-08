cmake_minimum_required(VERSION 3.8)
project(cat-sound-cls)

set(CMAKE_CXX_STANDARD 17)

add_executable(wav2mfcc-client
    src/list_ports_linux.cc
    src/wav2mfcc-client.cpp
    src/serial.cc
    src/unix.cc
)

add_executable(wav-monitor
    src/list_ports_linux.cc
    src/monitor.cpp
    src/serial.cc
    src/unix.cc
)

target_include_directories(wav2mfcc-client PRIVATE include)
target_include_directories(wav-monitor PRIVATE include)
