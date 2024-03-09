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
#include "usb_device.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #define __FPU_PRESENT 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "arm_math.h"
#include "cat_sound_cls.h"
#include "cat_sound_cls_data.h"
#include "usbd_cdc_if.h"
#include "st7789.h"
#include "mfcc.h"
#include "ppbuf.h"
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2S_BUFFER1_SIZE      (4096) // unit:byte
#define I2S_BUFFER2_SIZE      (4096) // unit:byte


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

#define EXAM_FLASH_OK         (0X01)
#define EXAM_AI_OK            (0X02)
#define EXAM_I2S_OK           (0X04)
#define EXAM_LCD_OK           (0X08)
#define EXAM_KEY_OK           (0X10)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char debug_print_buffer[128];
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
AI_ALIGNED(4) ai_u8 activations[AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZE]__attribute((section(".ccmram")))={0};

// Buffers used to store input and output tensors
AI_ALIGNED(4) float in_data[AI_CAT_SOUND_CLS_IN_1_SIZE]__attribute((section(".ccmram")))={0};
AI_ALIGNED(4) float out_data[AI_CAT_SOUND_CLS_OUT_1_SIZE]__attribute((section(".ccmram")))={0};

// Pointer to our model
ai_handle cat_sound_cls_model = AI_HANDLE_NULL;

// Initialize wrapper structs that hold pointers to data and info about the
// data (tensor height, width, channels)
ai_buffer ai_input[AI_CAT_SOUND_CLS_IN_NUM]__attribute((section(".ccmram")))={0};
ai_buffer ai_output[AI_CAT_SOUND_CLS_OUT_NUM]__attribute((section(".ccmram")))={0};

// Set working memory and get weights/biases from model
ai_network_params ai_params __attribute((section(".ccmram")))={0};

uint8_t i2s_buffer_1[I2S_BUFFER1_SIZE]; // real audio data is quarter of the buffer.
int16_t audio[256*160]={0};
uint32_t sample_count = 0;

volatile uint8_t state_key = 0;
volatile uint8_t state_i2s = 0;

uint8_t exam_result = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


  /**
   * receive MFCCs from PC
   */
  CDC_Receive((uint8_t*)in_data, AI_CAT_SOUND_CLS_IN_1_SIZE_BYTES);
  ai_cat_sound_cls_run(cat_sound_cls_model, &ai_input[0], &ai_output[0]);
  CDC_Transmit_FS((uint8_t*)out_data, AI_CAT_SOUND_CLS_OUT_1_SIZE_BYTES);
  
  mfcc_init();
  if(W25qxx_Init())
  {
    exam_result |= EXAM_FLASH_OK;
  }
  /**
   * @brief Send W25Q128 initialization result to PC
   * 
   */
  CDC_Transmit_FS(&exam_result, 1);
  lcddrv_Init();
  exam_result |= EXAM_LCD_OK;
  lcddrv_DispOn();
  lcddrv_FillColorPre(0,0,LCD_WIDTH,LCD_HEIGHT);

  /**
   * @brief send LCD initialization result to PC
   * 
   */
  CDC_Transmit_FS(&exam_result, 1);


  debug_lcd_printf("cat sound classification");
  ai_error err = ai_cat_sound_cls_create(&cat_sound_cls_model, (ai_buffer*)AI_CAT_SOUND_CLS_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE)
  {
    debug_lcd_printf("Neural network creation error - type = %lu, code = %lu\r\n", err.type, err.code);
    while(1){}
  }
  ai_network_params params;
  params.params = AI_CAT_SOUND_CLS_DATA_WEIGHTS(ai_cat_sound_cls_data_weights_get());
  params.activations = AI_CAT_SOUND_CLS_DATA_ACTIVATIONS(activations);

  if (!ai_cat_sound_cls_init(cat_sound_cls_model, &params))
  {
    debug_lcd_printf("Error: could not initialize network.");
  }
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_input[0].format = AI_BUFFER_FORMAT_FLOAT;
  ai_output[0].data = AI_HANDLE_PTR(out_data);
  exam_result |= EXAM_AI_OK;
  /**
   * @brief send AI initialization result to PC
   */
  CDC_Transmit_FS(&exam_result, 1);

  /**
   * wait for key to trigger start
   */
  while(state_key != SIGNAL_KEY0_RELEASE){}
  exam_result |= EXAM_KEY_OK;
  /* reset state machine */
  state_key = 0;
  /* reset audio size */
  sample_count = 0;
  /**
   * @brief send key status to PC
   */
  CDC_Transmit_FS(&exam_result, 1);

  /* Blue LED on */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

  /* LCD display shows string */
  debug_lcd_printf("start recording");

  while(sample_count < sizeof(audio)/sizeof(int16_t))
  {
    /**
     * division by 4 because DMA transfer frame in word(32 bits),
     * but capacity is in byte.
     * a sample contains 4 frames(2 for left channel, 2 for right channel),
     * and a frame contains 4 bytes(DMA frame size is set to WORD).
     */
    HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)i2s_buffer_1, sizeof(i2s_buffer_1)/sizeof(uint32_t));
    while(state_i2s != SIGNAL_I2S_RECV_DONE){}
    state_i2s = 0;
    exam_result |= EXAM_I2S_OK;
    /**
     * @brief send I2S status to PC
     */
    static uint8_t i2s_status_send_flag = 0;
    if(i2s_status_send_flag == 0)
    {
      CDC_Transmit_FS(&exam_result, 1);
      i2s_status_send_flag = 1;
    }

    uint16_t new_sample = sizeof(i2s_buffer_1)/sizeof(uint32_t)/4;
    int16_t *pconvert = (int16_t*)i2s_buffer_1;
    for(uint32_t i=0 ; i<new_sample ; i++)
    {
        audio[sample_count+i] = *(pconvert+i*8);
    }
    sample_count += new_sample;

    if(sample_count >= sizeof(audio)/sizeof(int16_t))
    {
        int32_t remaining_bytes = sizeof(audio);
        while(remaining_bytes > 0)
        {
            CDC_Transmit_FS(((uint8_t*)audio)+(sizeof(audio)-remaining_bytes), 256);
            remaining_bytes -= 256;
        }
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    }
  }
  W25qxx_EraseSector(0);
  W25qxx_WriteSector((uint8_t*)audio, 0, 0, FLASH_SECTOR_CAPACITY);
  W25qxx_ReadSector((uint8_t*)i2s_buffer_1, 0, 0, FLASH_SECTOR_CAPACITY);
  if(memcmp(audio, i2s_buffer_1, FLASH_SECTOR_CAPACITY) == 0)
  {
    exam_result |= EXAM_FLASH_OK;
  }
  else
  {
    exam_result &= ~EXAM_FLASH_OK;
  }
  CDC_Transmit_FS(&exam_result, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    

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
}

void debug_lcd_printf(char* fmt, ...)
{
  va_list argptr;
  memset(debug_print_buffer,0,sizeof(debug_print_buffer));
  va_start(argptr, fmt);
  vsprintf(debug_print_buffer, fmt, argptr);
  va_end(argptr);
  lcddrv_FillColorPre(0,0,LCD_WIDTH,LCD_HEIGHT);
  for(uint32_t i=0 ; i<LCD_WIDTH*LCD_HEIGHT; i++)
    ST7789_RAM = 0xFFFF;
  lcddrv_ShowString(0,0,LCD_WIDTH,LCD_HEIGHT,16,(uint8_t*)debug_print_buffer,0x0000);
}

void debug_printf(char* fmt, ...)
{
  va_list argptr;
  memset(debug_print_buffer,0,sizeof(debug_print_buffer));
  va_start(argptr, fmt);
  vsprintf(debug_print_buffer, fmt, argptr);
  va_end(argptr);
  CDC_Transmit_FS((uint8_t*)debug_print_buffer, strlen(debug_print_buffer));
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
