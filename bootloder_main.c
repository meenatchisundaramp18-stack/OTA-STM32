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
  ST_WAIT_HDR1 = 0,
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

#define BOOT_PIN                GPIO_PIN_0
#define BOOT_PIN_PORT           GPIOB

#define BOOTLOADER_START        0x08000000U
#define BOOTLOADER_SIZE         0x0000C000U      // 48KB

#define APP_START_ADDRESS       0x0800C000U
#define APP_MAX_SIZE            (464U * 1024U)

#define CHUNK_SIZE              512U
#define RX_DMA_BUF_SIZE         1024U

#define HEADER_BYTE1            0xAA
#define HEADER_BYTE2            0x55
#define END_MARKER              0x5A
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

static bl_state_t bl_state = ST_WAIT_HDR1;

static uint8_t  size_bytes[4];
static uint8_t  size_idx = 0;

static uint8_t  chunk_buffer[CHUNK_SIZE];
static uint16_t chunk_fill = 0;

static uint32_t fw_size = 0;
static uint32_t written = 0;

static volatile uint8_t upload_done  = 0;
static volatile uint8_t upload_error = 0;
static volatile uint8_t upload_active = 0;
static volatile uint8_t force_upload_mode = 0;
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

static void Enter_Upload_Mode(void);
static void Try_Jump_To_App_Or_Fallback(void);

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

static void BootRx_ResetUpload(void)
{
  fw_size = 0;
  written = 0;
  chunk_fill = 0;
  size_idx = 0;

  upload_done = 0;
  upload_error = 0;

  bl_state = ST_WAIT_HDR1;
}

/* Start circular DMA RX (WITH status + flag clear) */
static void BootRx_Start(void)
{
  rx_old_pos = 0;

  // Clear UART error flags before enabling DMA RX
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  __HAL_UART_CLEAR_PEFLAG(&huart1);

  HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
  if (st != HAL_OK)
  {
    UART_Print("ERR: HAL_UART_Receive_DMA failed\r\n");
  }
  else
  {
    UART_Print("DMA_RX_OK\r\n");
  }

  // IDLE interrupt optional (polling still works without it)
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static void BootRx_Check(void)
{
  if (huart1.hdmarx == NULL)
    return;

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

static void BootRx_Process(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t b = data[i];

    switch (bl_state)
    {
      case ST_WAIT_HDR1:
        if (b == HEADER_BYTE1)
          bl_state = ST_WAIT_HDR2;
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
          BootRx_ResetUpload();
          UART_Print("RDY_HDR\r\n");
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
            BootRx_ResetUpload();
            UART_Print("RDY_HDR\r\n");
            break;
          }

          if (Flash_Erase_App_Sectors() != 0)
          {
            UART_Print("ERR: Erase failed\r\n");
            upload_error = 1;
            upload_active = 0;
            BootRx_ResetUpload();
            UART_Print("RDY_HDR\r\n");
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
              BootRx_ResetUpload();
              UART_Print("RDY_HDR\r\n");
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

          UART_Print("Stay in upload mode. Press RESET to run app.\r\n");
          BootRx_ResetUpload();
          UART_Print("RDY_HDR\r\n");
        }
        else
        {
          UART_Print("ERR: Invalid end marker\r\n");
          upload_error = 1;
          upload_active = 0;
          BootRx_ResetUpload();
          UART_Print("RDY_HDR\r\n");
        }
        break;

      default:
        BootRx_ResetUpload();
        UART_Print("RDY_HDR\r\n");
        break;
    }
  }
}


uint8_t Check_Application_Valid(void)
{
  uint32_t sp = *(__IO uint32_t*)APP_START_ADDRESS;
  uint32_t pc = *(__IO uint32_t*)(APP_START_ADDRESS + 4);

  if (sp < 0x20000000U || sp > 0x20020000U) return 0;
  if (pc < APP_START_ADDRESS || pc > (APP_START_ADDRESS + APP_MAX_SIZE) || (pc & 1U) == 0U) return 0;

  return 1;
}

uint8_t Flash_Erase_App_Sectors(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t error;

  UART_Print("Erasing application sectors...\r\n");

  HAL_FLASH_Unlock();

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector       = FLASH_SECTOR_3;
  erase.NbSectors    = 5;

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
  HAL_Delay(100);

  __disable_irq();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  for (int i = 0; i < 8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  HAL_UART_DeInit(&huart1);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  SCB->VTOR = APP_START_ADDRESS;
  __set_MSP(sp);

  __DSB();
  __ISB();

  void (*app_reset_handler)(void) = (void (*)(void))(pc);
  app_reset_handler();

  while (1);
}

static void Enter_Upload_Mode(void)
{
  MX_DMA_Init();
  MX_USART1_UART_Init();

  UART_Print("\r\n==========================================\r\n");
  UART_Print("  UPLOAD MODE (PB0)\r\n");
  UART_Print("  APP @ 0x0800C000\r\n");
  UART_Print("==========================================\r\n");

  BootRx_ResetUpload();
  BootRx_Start();

  upload_active = 1;
  UART_Print("RDY_HDR\r\n");
}

static void Try_Jump_To_App_Or_Fallback(void)
{
  if (Check_Application_Valid())
  {
    Jump_To_Application();
  }
  else
  {
    Enter_Upload_Mode();
  }
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
  // PB0 pull-up, pressed => LOW
    if (HAL_GPIO_ReadPin(BOOT_PIN_PORT, BOOT_PIN) == GPIO_PIN_RESET)
      force_upload_mode = 1;

    HAL_Delay(50);
    LED_Blink(2, 80);

    if (force_upload_mode)
      Enter_Upload_Mode();
    else
      Try_Jump_To_App_Or_Fallback();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Upload mode loop (stays here until reset)
	    BootRx_Check();
	    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	    HAL_Delay(150);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
