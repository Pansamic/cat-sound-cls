/**
 * @file mfcc.cpp
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <unistd.h>
#include "serial/serial.h"
#include "mfcc/mfcc_params.h"



namespace fs = std::filesystem;

/**
 * @brief Extract single channel wav file and save it to a new file.
 * 
 * @param input The input wav file path.
 * @param output The output file path.
 * @return int 0 if success, 1 if error.
 */
int extract_wav(fs::path input, fs::path output)
{
    char header[44];
    uint16_t channels; // number of channels
    uint32_t sample_rate; // sample rate
    uint16_t bits_per_sample; // bits per sample
    uint32_t bytes_per_sample; // bytes per sample

    std::ifstream fin(input, std::ios::binary);
    std::ofstream fout(output, std::ios::out|std::ios::binary);
    // Check if the files are open
    if (!fin.is_open())
    {
        std::cerr << "Error opening wav file!" << std::endl;
        return 1;
    }
    if(!fout.is_open())
    {
        std::cerr << "Error opening output file!" << std::endl;
        fin.close();
        return 1;
    }

    /* read basic info of wav file */
    fin.seekg(0, std::ios::beg);
    fin.read(header, 44);
    channels = *(uint16_t *)(header + 22);
    sample_rate = *(uint32_t *)(header + 24);
    bits_per_sample = *(uint16_t *)(header + 34);
    bytes_per_sample = bits_per_sample / 8;

    std::streamsize data_start_pos = 44;
    fin.seekg(0, std::ios::end);
    std::streamsize data_end_pos = fin.tellg();
    fin.seekg(data_start_pos, std::ios::beg);
    std::streamsize data_size = data_end_pos - data_start_pos;
    char *data = new char[data_size];
    fin.read(data, data_size);

    // printf("file: %s\n", input.c_str());
    // printf("channels: %d\n", channels);
    // printf("sample rate: %d\n", sample_rate);
    // printf("bits per sample: %d\n", bits_per_sample);
    // printf("bytes per sample: %d\n", bytes_per_sample);
    // printf("data size: %ld\n", data_size);

    if(channels == 1)
    {
        
        fout.write(data, data_size);
    }
    else
    {
        if(channels == 2 && data_size & 1 == 1)
        {
            printf("Error: stereo channel but data size is not even\n");
            return 1;
        }
        else
        {
            for(int i = 0; i < data_size; i += channels * bytes_per_sample)
            {
                fout.write(data + i, bytes_per_sample);
            }
        }
    }
    delete[] data;
    fin.close();
    fout.close();
    return 0;
}



char* get_cmd_option(char **begin, char **end, const std::string &value)
{
    char **iter = std::find(begin, end, value);
    if (iter != end && ++iter != end)
        return *iter;
    return nullptr;
}

#define S_(x) #x
void progressbar(unsigned percent) 
{
  char pbstr[64];
  memset(pbstr, '#', 64);
  fprintf(stderr, "\r[%-" S_(64) ".*s] %u%%", percent * 64 / 100, pbstr, percent);
  if(percent == 100) fprintf(stderr, "\n");
}

int main(int argc, char* argv[])
{
    std::string USAGE = "mfcc : mfcc extractor client\n";
    USAGE += "Usage: mfcc [options] <input> <output>\n";
    USAGE += "Options:\n";
    USAGE += "  -h, --help\t\tShow this help message and exit\n";
    USAGE += "  -d, --dev\t\tSerial port\n";
    USAGE += "  -b, --baud\t\tBaud rate\n";
    USAGE += "  -p, --path\t\twork path, including 'audio' folder\n";
    USAGE += "USAGE EXAMPLES:\n";
    USAGE += "  wav2mfcc-client -d /dev/ttyACM0 -b 115200 -p /home/user/sound-cls\n";

    char **help = std::find(argv, argv+argc, "-h");
    if(help != argv+argc || std::find(argv, argv+argc, "--help") != argv+argc)
    {
        std::cout << USAGE << std::endl;
        return 0;
    }
    char* dev = get_cmd_option(argv, argv+argc, "-d");
    if(dev == nullptr) dev = get_cmd_option(argv, argv+argc, "--dev");
    char* baud = get_cmd_option(argv, argv+argc, "-b");
    if(baud == nullptr) baud = get_cmd_option(argv, argv+argc, "--baud");
    char* path = get_cmd_option(argv, argv+argc, "-p");
    if(path == nullptr) path = get_cmd_option(argv, argv+argc, "--path");

    if(dev == nullptr || baud == nullptr || path == nullptr)
    {
        std::cerr << "Error: missing arguments!" << std::endl;
        std::cerr << USAGE << std::endl;
        return 1;
    }

    serial::Serial ser(dev, std::stoi(baud), serial::Timeout::simpleTimeout(0xFFFFFFFF));

    if(!ser.isOpen())
    {
        std::cerr << "Error: failed to open serial port!" << std::endl;
        return 1;
    }

    fs::path root_path(path);
    fs::path audio_path = root_path / "audio";
    fs::path raw_path = root_path / "raw";
    fs::path mfcc_path = root_path / "mfcc";

    for (const auto& entry : fs::recursive_directory_iterator(audio_path))
    {
        if (!entry.is_regular_file() || entry.path().extension() != ".wav")
        {
            continue;
        }
        std::cout << "Processing file: " << entry.path() << std::endl;
        // Construct the relative path to maintain the folder structure
        auto relative_path = fs::relative(entry.path(), audio_path);
        fs::path dat_file_path = raw_path / relative_path;
        fs::path mfcc_file_path = mfcc_path / relative_path;
        dat_file_path.replace_extension(".dat");
        mfcc_file_path.replace_extension(".mfcc");
        // Create directories if they don't exist
        fs::create_directories(dat_file_path.parent_path());
        fs::create_directories(mfcc_file_path.parent_path());
        if (extract_wav(entry.path(), dat_file_path) != 0) 
        {
            std::cerr << "Error processing file: " << entry.path() << std::endl;
        }
        std::cout << "Extract success: " << dat_file_path << std::endl;

        /* send frames to MCU server */
        unsigned int offset = 1024;
        unsigned int raw_data_size = fs::file_size(dat_file_path);
        unsigned int hop_size = (raw_data_size - offset - FRAME_SIZE) / (NUM_FRAME-1);
        std::ifstream fin(dat_file_path, std::ios::binary);
        std::ofstream fout(mfcc_file_path, std::ios::out|std::ios::binary);
        // Check if the files are open
        if (!fin.is_open())
        {
            std::cerr << "Error opening wav file!" << std::endl;
            return 1;
        }
        if(!fout.is_open())
        {
            std::cerr << "Error opening output file!" << std::endl;
            fin.close();
            return 1;
        }
        uint16_t frame[FRAME_SIZE];
        float mfcc[NUM_MFCC];
        for(int i = 0; i < NUM_FRAME; i++)
        {
            fin.seekg(offset + i * hop_size, std::ios::beg);
            fin.read((char*)frame, FRAME_SIZE*sizeof(uint16_t));
            ser.write((uint8_t*)frame, FRAME_SIZE*sizeof(uint16_t));
            ser.read((uint8_t*)mfcc, NUM_MFCC*sizeof(float));
            fout.write((char*)mfcc, NUM_MFCC*sizeof(float));
            progressbar((i+1) * 100 / NUM_FRAME);
        }
        fin.close();
        fout.close();
    }

    ser.close();

    return 0;
}