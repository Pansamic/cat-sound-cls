/**
 * @file mfcc.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief MFCC extractor definition
 * @version 0.1
 * @date 2023-11-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MFCC_H__
#define __MFCC_H__

#include "stdint.h"

#define FRAME_SIZE           (1024)       // unit:word
#define FRAME_SIZE_IN_BYTES  (4096)       // unit:byte
#define NUM_FILTERS          (20)         // the number of mel filters
#define LOW_FREQ             (300)        // the lower frequency limit in Hz
#define HIGH_FREQ            (8000)       // the upper frequency limit in Hz
#define SAMPLING_RATE        (44100)      // the sampling rate in Hz
#define FFT_SIZE             (FRAME_SIZE) // the FFT size
#define FRAME_COUNT          (200)        // the number of frames
#define MFCC_CNT_PER_FRAME   (13)         // the number of MFCC coefficients

void mfcc_init(void);
void audio_get_mfcc(float* audio_src_ptr,float* audio_dst_ptr,float* mfcc_dst_ptr);

#endif /* __MFCC_H__ */
