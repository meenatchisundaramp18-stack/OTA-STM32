/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ST_WAIT_CMD = 0,
  ST_WAIT_HDR1,
  ST_WAIT_HDR2,
  ST_WAIT_SIZE,
  ST_RECV_DATA,
  ST_WAIT_END
} bl_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN                 GPIO_PIN_8
#define LED_PORT                GPIOB

#define BOOTLOADER_START        0x08000000U
#define BOOTLOADER_SIZE         0x0000C000U      // 48KB

#define APP_START_ADDRESS       0x0800C000U      // Start of sector 3 on STM32F4 [web:149]
#define APP_MAX_SIZE            (464U * 1024U)   // 512KB - 48KB = 464KB [web:130]

#define CHUNK_SIZE              512U

#define HEADER_BYTE1            0xAA
#define HEADER_BYTE2            0x55
#define END_MARKER              0x5A

#define BOOTLOADER_TIMEOUT_MS   5000U

#define RX_DMA_BUF_SIZE         1024U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
static uint8_t  rx_dma_buf[RX_DMA_BUF_SIZE];
static volatile uint16_t rx_old_pos = 0;

static bl_state_t bl_state = ST_WAIT_CMD;

static uint8_t size_bytes[4];
static uint8_t size_idx = 0;

static uint8_t chunk_buffer[CHUNK_SIZE];
static uint16_t chunk_fill = 0;

static uint32_t fw_size = 0;
static uint32_t written = 0;

static volatile uint8_t upload_done = 0;
static volatile uint8_t upload_error = 0;
static volatile uint8_t upload_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LED_Blink(uint8_t count, uint16_t delay_ms);
static void UART_Print(const char *s);

static void BootRx_Start(void);
static void BootRx_Check(void);
static void BootRx_Process(const uint8_t *data, uint16_t len);
static void BootRx_ResetUpload(void);

uint8_t Check_Application_Valid(void);
uint8_t Flash_Erase_App_Sectors(void);
uint8_t Flash_Write_Chunk(uint32_t address, uint8_t *data, uint32_t size);
void Jump_To_Application(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LED_Blink(uint8_t count, uint16_t delay_ms)
{
  for (uint8_t i = 0; i < count; i++)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(delay_ms);
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(delay_ms);
  }
}

static void UART_Print(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 1000);
}

/* ----------- DMA Circular RX Start ----------- */
static void BootRx_Start(void)
{
  rx_old_pos = 0;

  HAL_UART_Receive_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);

  // Enable IDLE line interrupt (make sure USART1 IRQ enabled in NVIC)
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* ----------- Check ring and process new bytes ----------- */
static void BootRx_Check(void)
{
  uint16_t pos = (uint16_t)(RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));

  if (pos != rx_old_pos)
  {
    if (pos > rx_old_pos)
    {
      BootRx_Process(&rx_dma_buf[rx_old_pos], (uint16_t)(pos - rx_old_pos));
    }
    else
    {
      BootRx_Process(&rx_dma_buf[rx_old_pos], (uint16_t)(RX_DMA_BUF_SIZE - rx_old_pos));
      BootRx_Process(&rx_dma_buf[0], pos);
    }
    rx_old_pos = pos;
  }
}

static void BootRx_ResetUpload(void)
{
  fw_size = 0;
  written = 0;
  chunk_fill = 0;
  size_idx = 0;
  upload_done = 0;
  upload_error = 0;
}

static void BootRx_Process(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t b = data[i];

    switch (bl_state)
    {
      case ST_WAIT_CMD:
        if (b == 'U' || b == 'u')
        {
          UART_Print("\r\nUpload command received!\r\n");
          UART_Print("Waiting for firmware data...\r\n\r\n");

          BootRx_ResetUpload();
          upload_active = 1;

          UART_Print("RDY_HDR\r\n");
          bl_state = ST_WAIT_HDR1;
        }
        break;

      case ST_WAIT_HDR1:
        if (b == HEADER_BYTE1) bl_state = ST_WAIT_HDR2;
        else
        {
          UART_Print("ERR: Invalid header\r\n");
          upload_error = 1;
          upload_active = 0;
          bl_state = ST_WAIT_CMD;
        }
        break;

      case ST_WAIT_HDR2:
        if (b == HEADER_BYTE2)
        {
          UART_Print("RDY_SIZE\r\n");
          size_idx = 0;
          bl_state = ST_WAIT_SIZE;
        }
        else
        {
          UART_Print("ERR: Invalid header\r\n");
          upload_error = 1;
          upload_active = 0;
          bl_state = ST_WAIT_CMD;
        }
        break;

      case ST_WAIT_SIZE:
        size_bytes[size_idx++] = b;
        if (size_idx >= 4)
        {
          fw_size = (uint32_t)size_bytes[0]
                  | ((uint32_t)size_bytes[1] << 8)
                  | ((uint32_t)size_bytes[2] << 16)
                  | ((uint32_t)size_bytes[3] << 24);

          if (fw_size == 0 || fw_size > APP_MAX_SIZE)
          {
            UART_Print("ERR: Invalid size\r\n");
            upload_error = 1;
            upload_active = 0;
            bl_state = ST_WAIT_CMD;
            break;
          }

          if (Flash_Erase_App_Sectors() != 0)
          {
            UART_Print("ERR: Erase failed\r\n");
            upload_error = 1;
            upload_active = 0;
            bl_state = ST_WAIT_CMD;
            break;
          }

          UART_Print("RDY_DATA\r\n");
          written = 0;
          chunk_fill = 0;
          bl_state = ST_RECV_DATA;
        }
        break;

      case ST_RECV_DATA:
        if (written < fw_size)
        {
          chunk_buffer[chunk_fill++] = b;
          written++;

          if (chunk_fill == CHUNK_SIZE || written == fw_size)
          {
            uint32_t addr = APP_START_ADDRESS + (written - chunk_fill);

            if (Flash_Write_Chunk(addr, chunk_buffer, chunk_fill) != 0)
            {
              UART_Print("ERR: Flash write failed\r\n");
              upload_error = 1;
              upload_active = 0;
              bl_state = ST_WAIT_CMD;
              break;
            }

            UART_Print("ACK\r\n");
            chunk_fill = 0;

            if (written == fw_size)
            {
              UART_Print("RDY_END\r\n");
              bl_state = ST_WAIT_END;
            }
          }
        }
        break;

      case ST_WAIT_END:
        if (b == END_MARKER)
        {
          UART_Print("\r\nUPLOAD COMPLETE!\r\n");
          upload_done = 1;
          upload_active = 0;
          bl_state = ST_WAIT_CMD;
        }
        else
        {
          UART_Print("ERR: Invalid end marker\r\n");
          upload_error = 1;
          upload_active = 0;
          bl_state = ST_WAIT_CMD;
        }
        break;

      default:
        bl_state = ST_WAIT_CMD;
        break;
    }
  }
}

uint8_t Check_Application_Valid(void)
{
  uint32_t sp = *(__IO uint32_t*)APP_START_ADDRESS;
  uint32_t pc = *(__IO uint32_t*)(APP_START_ADDRESS + 4);

  // SRAM range for STM32F411: 128KB => 0x20000000..0x2001FFFF (your old check 0x20020000 is also OK)
  if (sp < 0x20000000U || sp > 0x20020000U) return 0;

  // Reset handler must be within application flash region and Thumb bit must be 1
  if (pc < APP_START_ADDRESS || pc > (APP_START_ADDRESS + APP_MAX_SIZE) || (pc & 1U) == 0U) return 0;

  return 1;
}

/* IMPORTANT: app now starts at SECTOR 3 (0x0800C000) [web:149] */
uint8_t Flash_Erase_App_Sectors(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t error;

  UART_Print("Erasing application sectors...\r\n");

  HAL_FLASH_Unlock();

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  // App region: sectors 3..7 (sector 3 is 16KB at 0x0800C000) [web:149]
  erase.Sector    = FLASH_SECTOR_3;
  erase.NbSectors = 5;

  if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    UART_Print("ERR: Erase failed\r\n");
    return 1;
  }

  HAL_FLASH_Lock();
  UART_Print("Erase complete\r\n");
  return 0;
}

uint8_t Flash_Write_Chunk(uint32_t address, uint8_t *data, uint32_t size)
{
  HAL_FLASH_Unlock();

  for (uint32_t i = 0; i < size; i += 4)
  {
    uint32_t word = 0xFFFFFFFFU;
    uint32_t remaining = (size - i >= 4U) ? 4U : (size - i);
    memcpy(&word, &data[i], remaining);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return 1;
    }
  }

  HAL_FLASH_Lock();
  return 0;
}

void Jump_To_Application(void)
{
  uint32_t sp = *(__IO uint32_t*)APP_START_ADDRESS;
  uint32_t pc = *(__IO uint32_t*)(APP_START_ADDRESS + 4);

  UART_Print("Jumping to application...\r\n");
  HAL_Delay(200);

  // Minimal deinit
  HAL_UART_DeInit(&huart1);

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  // Set vector table to new app base
  SCB->VTOR = APP_START_ADDRESS;

  __set_MSP(sp);

  __DSB();
  __ISB();

  void (*app_reset_handler)(void) = (void (*)(void))(pc);
  app_reset_handler();

  while (1);
}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
    LED_Blink(2, 120);

    UART_Print("\r\n==========================================\r\n");
    UART_Print("  STM32 UART BOOTLOADER (DMA CIRCULAR RX)\r\n");
    UART_Print("  BL size: 48KB, APP @ 0x0800C000\r\n");
    UART_Print("==========================================\r\n");

    UART_Print("Press 'U' to upload firmware\r\n");
    UART_Print("Or wait for auto-jump...\r\n\r\n");

    BootRx_Start();

    uint32_t start_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BootRx_Check();

	     HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	     HAL_Delay(150);

	     if (!upload_active && (HAL_GetTick() - start_time) > BOOTLOADER_TIMEOUT_MS)
	     {
	       UART_Print("Timeout reached\r\nChecking for valid application...\r\n");

	       if (Check_Application_Valid())
	       {
	         UART_Print("Valid application found\r\n");
	         HAL_Delay(100);
	         Jump_To_Application();
	       }
	       else
	       {
	         UART_Print("No valid application found\r\nStaying in bootloader mode\r\n\r\n");
	         start_time = HAL_GetTick();
	       }
	     }

	     if (upload_done)
	     {
	       UART_Print("Firmware upload successful!\r\n");
	       HAL_Delay(100);

	       if (Check_Application_Valid())
	       {
	         Jump_To_Application();
	       }
	       else
	       {
	         UART_Print("ERROR: Application verification failed!\r\n");
	         upload_done = 0;
	       }
	     }

	     if (upload_error)
	     {
	       UART_Print("Upload failed! Try again.\r\n");
	       upload_error = 0;
	     }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
