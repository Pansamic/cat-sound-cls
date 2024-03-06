/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2s.h"
#include "spi.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include "cat_sound_cls.h"
#include "cat_sound_cls_data.h"
#include "w25qxx.h"
#include "st7789.h"
#include "mfcc.h"
#include "ppbuf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2S_BUFFER1_SIZE      (2048) // unit:byte
#define I2S_BUFFER2_SIZE      (2048) // unit:byte


#define FLASH_CAPACITY        (0x01000000) // unit:byte
#define FLASH_BLOCK_CAPACITY  (0X00010000) // unit:byte
#define FLASH_SECTOR_CAPACITY (0X00001000) // unit:byte
#define FLASH_PAGE_CAPACITY   (0X00000100) // unit:byte
#define FLASH_MAX_RECV_COUNT  (FLASH_CAPACITY/FLASH_SECTOR_CAPACITY)

#define SIGNAL_I2S_RECV_DONE  (0X00000001)
#define SIGNAL_I2S_RECV_STOP  (0X00000002)
#define SIGNAL_KEY0_PRESS     (0X00000004)
#define SIGNAL_KEY0_RELEASE   (0X00000008)
#define SIGNAL_KEY1_PRESS     (0X00000010)
#define SIGNAL_KEY1_RELEASE   (0X00000020)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char debug_print_buffer[256];
const char* cat_sound_map[] = {
    "angry",
    "defense",
    "fighting",
    "happy",
    "huntingmind",
    "mating",
    "mothercall",
    "paining",
    "resting",
    "warning"
};

uint8_t ai_result = 0;

// Chunk of memory used to hold intermediate values for neural network
// AI_ALIGNED(4) ai_u8 activations[AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZE] __attribute__((section(".ccmram")));

// Buffers used to store input and output tensors
AI_ALIGNED(4) ai_u8 in_data[AI_CAT_SOUND_CLS_IN_1_SIZE_BYTES] __attribute__((section(".ccmram")));
AI_ALIGNED(4) ai_u8 out_data[AI_CAT_SOUND_CLS_OUT_1_SIZE_BYTES] __attribute__((section(".ccmram"))) ;

// Pointer to our model
ai_handle cat_sound_cls_model = AI_HANDLE_NULL;

// Initialize wrapper structs that hold pointers to data and info about the
// data (tensor height, width, channels)
ai_buffer* ai_input[AI_CAT_SOUND_CLS_IN_NUM];
ai_buffer* ai_output[AI_CAT_SOUND_CLS_OUT_NUM];

// Set working memory and get weights/biases from model
ai_network_params ai_params;

uint8_t i2s_buffer_1[I2S_BUFFER1_SIZE]; // real audio data is quarter of the buffer.
uint8_t i2s_buffer_2[I2S_BUFFER2_SIZE];
ppbuf_t i2s_ppbuf={0};
float audio[I2S_BUFFER1_SIZE/16];

uint32_t sample_count = 0;

uint8_t state_key = 0;
uint8_t state_i2s = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t get_network_result(uint32_t *arr, uint8_t size);
void process_audio(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  mfcc_init();
  W25qxx_Init();
  lcddrv_Init();
  lcddrv_DispOn();
  lcddrv_FillColorPre(0,0,LCD_WIDTH,LCD_HEIGHT);
  debug_lcd_printf("cat sound classification");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if((state_key & SIGNAL_KEY0_PRESS))
    {
        ppbuf_init(&i2s_ppbuf, i2s_buffer_1, sizeof(i2s_buffer_1), i2s_buffer_2, sizeof(i2s_buffer_2));
        // W25qxx_EraseChip();
        /* reset audio size */
        sample_count = 0;

        /**
         * division by 4 because DMA transfer frame in word(32 bits),
         * but capacity is in byte.
         * a sample contains 4 frames(2 for left channel, 2 for right channel),
         * and a frame contains 4 bytes(DMA frame size is set to WORD).
         */
        HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)ppbuf_get_input(&i2s_ppbuf),ppbuf_get_input_capacity(&i2s_ppbuf)/4);

        /* Blue LED on */
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

        /* LCD display shows string */
        debug_lcd_printf("start recording");

        /* reset state machine */
        state_key = 0;
    }
    else if((state_key & SIGNAL_KEY0_RELEASE))
    {
        state_i2s = SIGNAL_I2S_RECV_STOP;
        state_key = 0;
    }
    if(state_i2s == SIGNAL_I2S_RECV_DONE)
    {
        /* switch ping pong buffer and start new DMA transport */
        if(ppbuf_switch(&i2s_ppbuf))
        {
            printf("ppbuf switch fail\r\n");
        }
        if(ppbuf_empty_input(&i2s_ppbuf))
        {
            printf("ppbuf input empty fail\r\n");
        }
        /**
         * division by 4 because DMA transfer frame in word(32 bits),
         * but capacity is in byte.
         * a sample contains 4 frames(2 for left channel, 2 for right channel),
         * and a frame contains 4 bytes(DMA frame size is set to WORD).
         */
        HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)ppbuf_get_input(&i2s_ppbuf),ppbuf_get_input_capacity(&i2s_ppbuf)/4);

        /* process valid data from i2s recv buffer */
        int32_t *ptemp = (int32_t*)ppbuf_get_output(&i2s_ppbuf);

        /**
         * only keep the 16bits of one chaneel of a 
         *  sample(stereo channel, 4 uint32_t per sample).
         * 'ppbuf_get_output_size(&i2s_ppbuf)/16' is from 
         * 'ppbuf_get_output_size(&i2s_ppbuf)/4/4', where
         * division by 4 means only a quarter of the buffer is valid data,
         * division by 4 means DMA frame is set to 'Word'(4 bytes),
         */
        uint16_t new_sample = ppbuf_get_output_size(&i2s_ppbuf)/16;
        for(uint32_t i=0 ; i<new_sample ; i++)
        {
            audio[i] = (float)*((int16_t*)(ptemp + (i*4)));
        }
        uint16_t remaining_size = new_sample*4; // unit:byte
        uint16_t write_size = 0; // unit:byte
        while(remaining_size > 0)
        {
            write_size = (remaining_size>FLASH_PAGE_CAPACITY - (sample_count*4)%FLASH_PAGE_CAPACITY)?
                         (FLASH_PAGE_CAPACITY - (sample_count*4)%FLASH_PAGE_CAPACITY) : remaining_size;
            W25qxx_WritePage(
                ((uint8_t*)audio)+(new_sample*4-remaining_size),
                (sample_count*4)/FLASH_PAGE_CAPACITY,
                (sample_count*4)%FLASH_PAGE_CAPACITY,
                write_size);
            /**
             * division by 4 means the unit of sample_count is float(4 bytes) but the
             * unit of ppbuf_get_output_size(&i2s_ppbuf) is byte.
             */
            sample_count+=write_size/4; //notice type:float(4 bytes)
            remaining_size -= write_size;
        }

        /**
         * decrease the size of the output buffer after reading data 
         * from the output buffer.
         */
        if(ppbuf_decrease(&i2s_ppbuf, ppbuf_get_output_size(&i2s_ppbuf)))
        {
            printf("ppbuf decrease fail\r\n");
        }
        state_i2s = 0;
    }
    if (state_i2s == SIGNAL_I2S_RECV_STOP)
    {
        HAL_I2S_DMAStop(&hi2s2);

        // /**
        //  * hi2s2.RxXferCount is initialized at the same value as transfer size at the
        //  * beginning of the transfer and decremented when a sample is received
        //  * NbSamplesReceived = RxBufferSize-RxBufferCount
        //  */
        // uint16_t received_frames = hi2s2.hdmarx->Instance->NDTR/4;

        // /**
        //  * the final DMA transfer is over after the following line
        //  * is executed literally.
        //  */
        // ppbuf_increase(&i2s_ppbuf, received_frames*4);

        // /**
        //  * so, we switch the ping pong buffer and finish the last
        //  * DMA transfer.
        //  */
        // if(ppbuf_switch(&i2s_ppbuf))
        // {
        //     debug_printf("ppbuf switch fail\r\n");
        // }
        // int32_t* ptemp = (int32_t*)ppbuf_get_output(&i2s_ppbuf);

        // /**
        //  * only keep the 16bits of one chaneel of a 
        //  * sample(stereo channel, 4 uint32_t per sample) 
        //  */
        // for(uint32_t i=0 ; i<ppbuf_get_output_size(&i2s_ppbuf)/4 ; i++)
        // {
        //     audio[i] = (float)*((int16_t*)(ptemp + (i*4)));
        // }

        // uint16_t new_sample_count = received_frames*4; // unit:byte
        // uint16_t remaining_size = new_sample_count; // unit:byte
        // uint16_t write_size = 0; // unit:byte
        // while(remaining_size > 0)
        // {
        //     write_size = (remaining_size>FLASH_PAGE_CAPACITY - (sample_count*4)%FLASH_PAGE_CAPACITY)?
        //                  (FLASH_PAGE_CAPACITY - (sample_count*4)%FLASH_PAGE_CAPACITY) : remaining_size;
        //     W25qxx_WritePage(
        //         (uint8_t*)audio,
        //         (sample_count*4)/FLASH_PAGE_CAPACITY,
        //         (sample_count*4)%FLASH_PAGE_CAPACITY,
        //         write_size);
        //     /**
        //      * division by 4 means only a quarter of the buffer is valid data.
        //      * division by 4 means the unit of sample_count is float(4 bytes) but the
        //      * unit of ppbuf_get_output_size(&i2s_ppbuf) is byte.
        //      */
        //     sample_count+=write_size/4; //notice type:float(4 bytes)
        //     remaining_size -= write_size;
        // }

        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        // print_flash_audio();
        process_audio();
        state_i2s = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void process_audio(void)
{
    uint32_t frame_offset = (sample_count - FRAME_SIZE)/(FRAME_COUNT - 1)*4;
    float* mfcc_buffer = ai_input[0]->data;

    for(uint8_t i=0 ; i<FRAME_COUNT ; i++)
    {
        memset(i2s_buffer_1,0,sizeof(i2s_buffer_1));
        memset(i2s_buffer_2,0,sizeof(i2s_buffer_2));
        W25qxx_ReadBytes(i2s_buffer_1,i*frame_offset,FRAME_SIZE_IN_BYTES);
        audio_get_mfcc((float*)i2s_buffer_1, (float*)i2s_buffer_2, (mfcc_buffer+i*MFCC_CNT_PER_FRAME));
    }
    ai_cat_sound_cls_run(cat_sound_cls_model, ai_input[0], ai_output[0]);
    ai_float *out_data = (ai_float *)ai_output[0]->data;
    // debug_printf("cat_sound_cls: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n",out_data[0], out_data[1],out_data[2], out_data[3],out_data[4], out_data[5],out_data[6], out_data[7],out_data[8], out_data[9]);
    // debug_printf("cat_sound_cls: 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\r\n",out_data[0], out_data[1], out_data[2], out_data[3],out_data[4], out_data[5],out_data[6], out_data[7],out_data[8], out_data[9]);
    ai_result = get_network_result((uint32_t*)out_data, 10);
    debug_lcd_printf("CNN process result: %s",cat_sound_map[ai_result]);
}
uint8_t get_network_result(uint32_t *arr, uint8_t size)
{
  // Check if the array is empty
  if (size == 0) {
    return -1; // Return -1 to indicate an error
  }
  // Initialize the max value and index to the first element
  uint32_t max = arr[0];
  uint8_t max_idx = 0;
  // Loop through the rest of the array
  for (uint8_t i = 1; i < size; i++) {
    // If the current element is greater than the max value
    if (arr[i] > max) {
      // Update the max value and index
      max = arr[i];
      max_idx = i;
    }
  }
  // Return the max index
  return max_idx;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==KEY0_Pin)
    {
        if(KEY0_GPIO_Port->IDR & KEY0_Pin)
            state_key = SIGNAL_KEY0_RELEASE;
        else
            state_key = SIGNAL_KEY0_PRESS;
    }
    else if(GPIO_Pin==KEY1_Pin)
    {
        if(KEY1_GPIO_Port->IDR & KEY1_Pin)
            state_key = SIGNAL_KEY1_RELEASE;
        else
            state_key = SIGNAL_KEY1_PRESS;
    }
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    state_i2s = SIGNAL_I2S_RECV_DONE;
    ppbuf_increase(&i2s_ppbuf, ppbuf_get_input_capacity(&i2s_ppbuf));
}
//void debug_printf(char* fmt, ...)
//{
//	va_list argptr;
//	uint16_t length;
//    memset(debug_print_buffer,0,256);
//	va_start(argptr, fmt);
//	length = vsprintf(debug_print_buffer, fmt, argptr);
//	va_end(argptr);
//	HAL_UART_Transmit(&huart1, debug_print_buffer, length, HAL_MAX_DELAY);
//}

void debug_lcd_printf(char* fmt, ...)
{
	va_list argptr;
    memset(debug_print_buffer,0,256);
	va_start(argptr, fmt);
	vsprintf(debug_print_buffer, fmt, argptr);
	va_end(argptr);
	lcddrv_FillColorPre(0,0,LCD_WIDTH,LCD_HEIGHT);
	for(uint32_t i=0 ; i<LCD_WIDTH*LCD_HEIGHT; i++)
	    ST7789_RAM = 0xFFFF;
	lcddrv_ShowString(0,0,LCD_WIDTH,LCD_HEIGHT,16,(uint8_t*)debug_print_buffer,0x0000);
}
__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file;

    return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;

    return len;
}

__attribute__((weak)) int _isatty(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

__attribute__((weak)) int _close(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 0;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _lseek(int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    errno = EBADF;
    return -1;
}

__attribute__((weak)) int _fstat(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
