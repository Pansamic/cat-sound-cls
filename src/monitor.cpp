/**
 * @file monitor.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <fstream>
#include <serial/serial.h>


int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <serial port>" << std::endl;
        return 1;
    }
    char *audio_buffer = new char[256*78*4];
    serial::Serial ser(argv[1], 115200, serial::Timeout::simpleTimeout(0XFFFFFFFF));
    std::cout << argv[1] << ":Is the serial port open?";
    if (ser.isOpen())
        std::cout << " Yes." << std::endl;
    else
        std::cout << " No." << std::endl;
    ser.read((uint8_t*)audio_buffer, 256*78*4);
    std::ofstream sampled_audio("audio.raw", std::ios::binary);
    sampled_audio.write(audio_buffer, 256*78*4);
    free(audio_buffer);
    sampled_audio.close();
    ser.close();
    return 0;
}