/**
 * @file systest.cpp
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
#include <filesystem>
#include <algorithm>
#include <serial/serial.h>

class SysTester
{
private:
    /* MCU peripheral status */
    const unsigned char kFlashOk = 0x01;
    const unsigned char kAiOk = 0x02;
    const unsigned char kI2sOk = 0x04;
    const unsigned char kLcdOk = 0x08;
    const unsigned char kKeyOk = 0x10;

    /* audio processing params */
    const unsigned int kNumMfcc = 13;
    const unsigned int kNumFrame = 200;
    const unsigned int kNumFrameSize = 1024;
    const unsigned int kNumAudioSample = 256 * 160;

    const unsigned int kNumAiOutput = 10;

    /* serial port params */
    std::shared_ptr<serial::Serial> ser_;
    std::string port_;
    const unsigned int kBaudrate = 921600;
    const unsigned int kTimeout = 20000; // unit: ms

    /* exam output files */
    std::filesystem::path root_path_;
public:
    SysTester(char* port, char* root_path)
    : port_(port), root_path_(root_path)
    {
        ser_ = std::make_shared<serial::Serial>(port_, kBaudrate, serial::Timeout::simpleTimeout(kTimeout));
        if(!ser_->isOpen())
        {
            std::cerr << "Error: failed to open serial port!" << std::endl;
            return ;
        }
        else
        {
            std::cout << "open serial port: " << port_ << std::endl;
        }
    }
    ~SysTester()
    {
        ser_->close();
    }
    int CreateWavFromAudio(std::ofstream &file, int16_t *audio_buffer, int size)
    {
        char header[44];
        uint16_t channels = 1; // number of channels
        uint32_t sample_rate = 22050; // sample rate
        uint16_t bits_per_sample = 16; // bits per sample
        uint32_t bytes_per_sample = bits_per_sample / 8;
        uint32_t data_size = size * bytes_per_sample;
        uint32_t file_size = data_size + 44 - 8;
        uint32_t byte_rate = sample_rate * channels * bytes_per_sample;
        uint16_t block_align = channels * bytes_per_sample;
        // write header
        header[0] = 'R';
        header[1] = 'I';
        header[2] = 'F';
        header[3] = 'F';
        *(uint32_t *)(header + 4) = file_size;
        header[8] = 'W';
        header[9] = 'A';
        header[10] = 'V';
        header[11] = 'E';
        header[12] = 'f';
        header[13] = 'm';
        header[14] = 't';
        header[15] = ' ';
        *(uint32_t *)(header + 16) = 16;
        *(uint16_t *)(header + 20) = 1;
        *(uint16_t *)(header + 22) = channels;
        *(uint32_t *)(header + 24) = sample_rate;
        *(uint32_t *)(header + 28) = byte_rate;
        *(uint16_t *)(header + 32) = block_align;
        *(uint16_t *)(header + 34) = bits_per_sample;
        header[36] = 'd';
        header[37] = 'a';
        header[38] = 't';
        header[39] = 'a';
        *(uint32_t *)(header + 40) = data_size;
        file.write(header, 44);
        file.write((char *)audio_buffer, data_size);
        return 0;
    }
    int CreateTxtFromAudio(std::ofstream &file, int16_t *audio_buffer, int size)
    {
        std::string audio_str;
        for(unsigned int i=0 ; i<size ; i++)
        {
            audio_str += std::to_string(audio_buffer[i]) + "\n";
        }
        file.write(audio_str.c_str(), audio_str.size());
        return 0;
    }
    int ExamDevStatus(unsigned char status)
    {
        unsigned char exam_status = 0;
        size_t ret = ser_->read(&exam_status, 1);
        if(ret != 1)
        {
            return 1;
        }
        if (exam_status&status!=status)
        {
            return 2;
        }
        return 0;
    }
    
    int ExamAudio()
    {
        int16_t *audio_buffer_ptr = new int16_t[kNumAudioSample];
        std::cout << "Recording until green light on, PC is receiving data..." << std::endl;
        size_t recv_size = ser_->read((uint8_t*)audio_buffer_ptr, kNumAudioSample*sizeof(int16_t));
        if(recv_size != kNumAudioSample*sizeof(int16_t))
        {
            std::cerr << "Error: Receive timeout. received " << std::to_string(recv_size) << " bytes.";
            std::cerr << "Expected " << std::to_string(kNumAudioSample*sizeof(int16_t)) << " bytes." << std::endl;
            std::cout << "Red lights on board means hardware error." << std::endl;
            return -1;
        }
        std::ofstream audio_ascii(root_path_/"audio.txt", std::ios::binary);
        std::ofstream audio_wav(root_path_/"audio.wav", std::ios::binary);
        CreateTxtFromAudio(audio_ascii, audio_buffer_ptr, kNumAudioSample);
        CreateWavFromAudio(audio_wav, audio_buffer_ptr, kNumAudioSample);
        audio_ascii.close();
        audio_wav.close();
        delete [] audio_buffer_ptr;
        return 0;
    }
    int ExamAiOutput()
    {
        float* ai_output_ptr = new float[kNumAiOutput];
        ser_->read((uint8_t*)ai_output_ptr, kNumAiOutput*sizeof(float));
        std::cout << "AI output: ";
        /* convert float to string and input to exam file */
        for(unsigned int i=0 ; i<kNumAiOutput ; i++)
        {
            std::cout << std::to_string(ai_output_ptr[i]) << " ";
        }
        std::cout << std::endl;
        delete [] ai_output_ptr;
        return 0;
    }
    int SendMfcc()
    {
        float *mfcc_ptr = new float[kNumMfcc * kNumFrame];
        std::filesystem::path mfcc_file_path = root_path_ / "mfcc/test/Angry/angry_1.mfcc";
        std::ifstream fmfcc(mfcc_file_path);
        fmfcc.read((char*)mfcc_ptr, kNumMfcc * kNumFrame * sizeof(float));
        ser_->write((uint8_t*)mfcc_ptr, kNumMfcc * kNumFrame * sizeof(float));
        delete [] mfcc_ptr;
        std::cout << "MFCC is sent, " << kNumMfcc * kNumFrame << " params in file:" << mfcc_file_path << std::endl;
        fmfcc.close();
        return 0;
    }
    int run()
    {
        int ret = 0;

        SendMfcc();

        ExamAiOutput();
        ret = ExamDevStatus(kFlashOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive FLash status." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: Flash initialization failed." << std::endl;
            std::cout << "Is flash chip or MCU not tightly soldered?" << std::endl;
        }
        else // correct
        {
            std::cout << "Flash initialize done." << std::endl;
        }
        ret = ExamDevStatus(kLcdOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive LCD status." << std::endl;
            std::cout << "Please check LCD config or USB connection." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: LCD is not working." << std::endl;
        }
        else // correct
        {
            std::cout << "LCD is working." << std::endl;
        }

        ret = ExamDevStatus(kAiOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive AI status." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: AI is not working." << std::endl;
            std::cout << "Please use supported MCU, STM32F407ZGT6 or APM32F407ZGT6-C2" << std::endl;
            std::cout << "please don't modify AI related code." << std::endl;
        }
        else // correct
        {
            std::cout << "AI init done." << std::endl;
        }
        /* need key press */
        std::cout << "Now please click Key0 on board..." << std::endl;

        ret = ExamDevStatus(kKeyOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive Key status." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: key is not working." << std::endl;
            std::cout << "Is key or MCU not tightly soldered?" << std::endl;
        }
        else // correct
        {
            std::cout << "Key is working." << std::endl;
        }
        ret = ExamDevStatus(kI2sOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive I2S interface status." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: I2S is not working." << std::endl;
        }
        else // correct
        {
            std::cout << "I2S is working." << std::endl;
        }
        if(ExamAudio())
        {
            return -1;
        }
        std::cout << "Raw audio file is saved in audio.txt" << std::endl;
        std::cout << "Wav audio file is saved in audio.wav" << std::endl;

        ret = ExamDevStatus(kFlashOk | kAiOk | kI2sOk | kLcdOk | kKeyOk);
        if(ret == 1) // serial read() timeout
        {
            std::cerr << "TimeoutExecption: Can not receive confirmation status." << std::endl;
        }
        else if(ret == 2) // unexpected status
        {
            std::cerr << "WrongStatus: Some system components are not working." << std::endl;
        }
        else // correct
        {
            std::cout << "All system components are working." << std::endl;
        }
        return 0;
    }
};


char* get_cmd_option(char **begin, char **end, const std::string &value)
{
    char **iter = std::find(begin, end, value);
    if (iter != end && ++iter != end)
        return *iter;
    return nullptr;
}


int main(int argc, char **argv)
{
    std::string USAGE = "systest : cat-sound-cls eval board system test\n";
    USAGE += "Usage: systest -d <serial_dev> -p </path/to/cat-sound-cls>\n";
    USAGE += "Options:\n";
    USAGE += "  -h, --help\t\tShow this help message and exit\n";
    USAGE += "  -d, --dev\t\tSerial port\n";
    USAGE += "  -p, --path\t\twork path, including 'audio' folder\n";

    char **help = std::find(argv, argv + argc, "-h");
    if (help != argv + argc || std::find(argv, argv + argc, "--help") != argv + argc)
    {
        std::cout << USAGE << std::endl;
        return 0;
    }
    char *dev = get_cmd_option(argv, argv + argc, "-d");
    if (dev == nullptr)
        dev = get_cmd_option(argv, argv + argc, "--dev");
    char *path = get_cmd_option(argv, argv + argc, "-p");
    if (path == nullptr)
        path = get_cmd_option(argv, argv + argc, "--path");

    // std::shared_ptr<HWTester> hw_tester = std::make_shared<HWTester>(dev, path);
    SysTester sys_tester(dev, path);
    sys_tester.run();
    return 0;
}