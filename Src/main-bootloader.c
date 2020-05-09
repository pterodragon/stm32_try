/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "TI_aes_128.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ACK                 0x79
#define NACK                0x1F
#define CMD_ERASE           0x43
#define CMD_GETID           0x02
#define CMD_WRITE           0x2b

#define APP_START_ADDRESS   0x08004000 /* In STM32F446RE this corresponds with the start
                                        address of Sector 1 */

#define SRAM_SIZE           128*1024     // STM32F446RE has 128KB of RAM
#define SRAM_END            (SRAM_BASE + SRAM_SIZE)

#define ENABLE_BOOTLOADER_PROTECTION 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Global macros */
/* The AES_KEY cannot be defined const, because the aes_enc_dec() function
 temporarily modifies its content */
uint8_t AES_KEY[] = { 0x4D, 0x61, 0x73, 0x74, 0x65, 0x72, 0x69, 0x6E, 0x67,
                      0x20, 0x20, 0x53, 0x54, 0x4D, 0x33, 0x32 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_GPIO_Deinit(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void _start(void);
void CHECK_AND_SET_FLASH_PROTECTION(void);
void cmdErase(uint8_t *pucData);
void cmdGetID(uint8_t *pucData);
void cmdWrite(uint8_t *pucData);
int main(void);
void SysTick_Handler();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Minimal vector table */
uint32_t *vector_table[] __attribute__((section(".isr_vector"))) = {
    (uint32_t *) SRAM_END, // initial stack pointer
    (uint32_t *) _start,   // _start is the Reset_Handler
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (uint32_t *) SysTick_Handler };

// Begin address for the initialization values of the .data section.
// defined in linker script
extern uint32_t _sidata;
// Begin address for the .data section; defined in linker script
extern uint32_t _sdata;
// End address for the .data section; defined in linker script
extern uint32_t _edata;
// Begin address for the .bss section; defined in linker script
extern uint32_t _sbss;
// End address for the .bss section; defined in linker script
extern uint32_t _ebss;

inline void
__attribute__((always_inline))
__initialize_data(uint32_t* from, uint32_t* region_begin, uint32_t* region_end) {
  // Iterate and copy word by word.
  // It is assumed that the pointers are word aligned.
  uint32_t *p = region_begin;
  while (p < region_end)
    *p++ = *from++;
}

inline void
__attribute__((always_inline))
__initialize_bss(uint32_t* region_begin, uint32_t* region_end) {
  // Iterate and copy word by word.
  // It is assumed that the pointers are word aligned.
  uint32_t *p = region_begin;
  while (p < region_end)
    *p++ = 0;
}

void __attribute__ ((noreturn,weak))
_start(void) {
  __initialize_data(&_sidata, &_sdata, &_edata);
  __initialize_bss(&_sbss, &_ebss);
  main();

  for (;;)
    ;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  // SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 1 */
  uint32_t ulTicks = 0;
  uint8_t ucUartBuffer[20];
  /* USER CODE END 1 */

#if ENABLE_BOOTLOADER_PROTECTION
  /* Ensures that the first sector of flash is write-protected preventing that the
   bootloader is overwritten */
  CHECK_AND_SET_FLASH_PROTECTION();
#endif

  /* USER CODE BEGIN 2 */
  /* If USER_BUTTON is pressed */
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
    /* CRC and UART2 peripherals enabled */
    MX_CRC_Init();
    MX_USART2_UART_Init();

    ulTicks = HAL_GetTick();

    while (1) {
      /* Every 500ms the LD2 LED blinks, so that we can see the bootloader running. */
      if (HAL_GetTick() - ulTicks > 500) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        ulTicks = HAL_GetTick();
      }

      /* We check for new commands arriving on the UART2 */
      HAL_UART_Receive(&huart2, ucUartBuffer, 20, 10);
      switch (ucUartBuffer[0]) {
      case CMD_GETID:
        cmdGetID(ucUartBuffer);
        break;
      case CMD_ERASE:
        cmdErase(ucUartBuffer);
        break;
      case CMD_WRITE:
        cmdWrite(ucUartBuffer);
        break;
      };
    }
  } else {
    /* USER_BUTTON is not pressed. We first check if the first 4 bytes starting from
     APP_START_ADDRESS contain the MSP (end of SRAM). If not, the LD2 LED blinks quickly. */
    if (*((uint32_t*) APP_START_ADDRESS) != SRAM_END) {
      while (1) {
        HAL_Delay(30);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
    } else {
      /* A valid program seems to exist in the second sector: we so prepare the MCU
       to start the main firmware */
      MX_GPIO_Deinit(); //Puts GPIOs in default state
      SysTick->CTRL = 0x0; //Disables SysTick timer and its related interrupt
      HAL_DeInit();

      RCC->CIR = 0x00000000; //Disable all interrupts related to clock
      __set_MSP(*((volatile uint32_t*) APP_START_ADDRESS)); //Set the MSP

      __DMB(); //ARM says to use a DMB instruction before relocating VTOR */
      SCB->VTOR = APP_START_ADDRESS; //We relocate vector table to the sector 1
      __DSB(); //ARM says to use a DSB instruction just after relocating VTOR */

      /* We are now ready to jump to the main firmware */
      uint32_t JumpAddress = *((volatile uint32_t*) (APP_START_ADDRESS + 4));
      void (*reset_handler)(void) = (void*)JumpAddress;
      reset_handler(); //We start the execution from he Reset_Handler of the main firmware

      for (;;)
        ; //Never coming here
    }
  }
  /* USER CODE END 2 */
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableOverDrive();

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */
  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */
  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void MX_GPIO_Deinit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  /* Warning : Reconfiguring all GPIO will close the connection with the debugger */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
}

/* Performs a flash erase of a given number of sectors/pages.
 *
 * An erase command has the following structure:
 *
 * ----------------------------------------
 * | CMD_ID | # of sectors      |  CRC32  |
 * | 1 byte |     1 byte        | 4 bytes |
 * |--------|-------------------|---------|
 * |  0x43  | N or 0xFF for all |   CRC   |
 * ----------------------------------------
 */
void cmdErase(uint8_t *pucData) {
  FLASH_EraseInitTypeDef eraseInfo;
  uint32_t ulBadBlocks = 0, ulCrc = 0;
  uint32_t pulCmd[] = { pucData[0], pucData[1] };

  memcpy(&ulCrc, pucData + 2, sizeof(uint32_t));

  /* Checks if provided CRC is correct */
  if (ulCrc == HAL_CRC_Calculate(&hcrc, pulCmd, 2) &&
      (pucData[1] > 0 && (pucData[1] < FLASH_SECTOR_TOTAL - 1 || pucData[1] == 0xFF))) {
    /* If data[1] contains 0xFF, it deletes all sectors; otherwise
     * the number of sectors specified. */
    eraseInfo.Banks = FLASH_BANK_1;
    eraseInfo.Sector = FLASH_SECTOR_1;
    eraseInfo.NbSectors = pucData[1] == 0xFF ? FLASH_SECTOR_TOTAL - 1 : pucData[1];
    eraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock(); //Unlocks the flash memory
    HAL_FLASHEx_Erase(&eraseInfo, &ulBadBlocks); //Deletes given sectors */
    HAL_FLASH_Lock(); //Locks again the flash memory

    /* Sends an ACK */
    pucData[0] = ACK;
    HAL_UART_Transmit(&huart2, (uint8_t *) pucData, 1, HAL_MAX_DELAY);
  } else {
    /* The CRC is wrong: sends a NACK */
    pucData[0] = NACK;
    HAL_UART_Transmit(&huart2, pucData, 1, HAL_MAX_DELAY);
  }
}

/* Retrieve the STM32 MCU ID
 *
 * A GET_ID command has the following structure:
 *
 * --------------------
 * | CMD_ID |  CRC32  |
 * | 1 byte | 4 bytes |
 * |--------|---------|
 * |  0x02  |   CRC   |
 * --------------------
 */
void cmdGetID(uint8_t *pucData) {
  uint16_t usDevID;
  uint32_t ulCrc = 0;
  uint32_t ulCmd = pucData[0];

  memcpy(&ulCrc, pucData + 1, sizeof(uint32_t));

  /* Checks if provided CRC is correct */
  if (ulCrc == HAL_CRC_Calculate(&hcrc, &ulCmd, 1)) {
    usDevID = (uint16_t) (DBGMCU->IDCODE & 0xFFF); //Retrieves MCU ID from DEBUG interface

    /* Sends an ACK */
    pucData[0] = ACK;
    HAL_UART_Transmit(&huart2, pucData, 1, HAL_MAX_DELAY);

    /* Sends the MCU ID */
    HAL_UART_Transmit(&huart2, (uint8_t *) &usDevID, 2, HAL_MAX_DELAY);
  } else {
    /* The CRC is wrong: sends a NACK */
    pucData[0] = NACK;
    HAL_UART_Transmit(&huart2, pucData, 1, HAL_MAX_DELAY);
  }
}

/* Performs a write of 16 bytes starting from the specified address.
 *
 * A write command has the following structure:
 *
 * ----------------------------------------
 * | CMD_ID | starting address  |  CRC32  |
 * | 1 byte |     4 byte        | 4 bytes |
 * |--------|-------------------|---------|
 * |  0x2b  |    0x08004000     |   CRC   |
 * ----------------------------------------
 *
 * The second message has the following structure
 *
 * ------------------------------
 * |    data bytes    |  CRC32  |
 * |      16 bytes    | 4 bytes |
 * |------------------|---------|
 * | BBBBBBBBBBBBBBBB |   CRC   |
 * ------------------------------
 */
void cmdWrite(uint8_t *pucData) {
  uint32_t ulSaddr = 0, ulCrc = 0;

  memcpy(&ulSaddr, pucData + 1, sizeof(uint32_t));
  memcpy(&ulCrc, pucData + 5, sizeof(uint32_t));

  uint32_t pulData[5];
  for(int i = 0; i < 5; i++)
    pulData[i] = pucData[i];

  /* Checks if provided CRC is correct */
  if (ulCrc == HAL_CRC_Calculate(&hcrc, pulData, 5) && ulSaddr >= APP_START_ADDRESS) {
    /* Sends an ACK */
    pucData[0] = ACK;
    HAL_UART_Transmit(&huart2, (uint8_t *) pucData, 1, HAL_MAX_DELAY);

    /* Now retrieves given amount of bytes plus the CRC32 */
    if (HAL_UART_Receive(&huart2, pucData, 16 + 4, 200) == HAL_TIMEOUT)
      return;

    memcpy(&ulCrc, pucData + 16, sizeof(uint32_t));

    /* Checks if provided CRC is correct */
    if (ulCrc == HAL_CRC_Calculate(&hcrc, (uint32_t*) pucData, 4)) {
      HAL_FLASH_Unlock(); //Unlocks the flash memory

      /* Decode the sent bytes using AES-128 ECB */
      aes_enc_dec((uint8_t*) pucData, AES_KEY, 1);
      for (uint8_t i = 0; i < 16; i++) {
        /* Store each byte in flash memory starting from the specified address */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, ulSaddr, pucData[i]);
        ulSaddr += 1;
      }
      HAL_FLASH_Lock(); //Locks again the flash memory

      /* Sends an ACK */
      pucData[0] = ACK;
      HAL_UART_Transmit(&huart2, (uint8_t *) pucData, 1, HAL_MAX_DELAY);
    } else {
      goto sendnack;
    }
  } else {
    goto sendnack;
  }

sendnack:
  pucData[0] = NACK;
  HAL_UART_Transmit(&huart2, (uint8_t *) pucData, 1, HAL_MAX_DELAY);
}

void CHECK_AND_SET_FLASH_PROTECTION(void) {
  FLASH_OBProgramInitTypeDef obConfig;

  /* Retrieves current OB */
  HAL_FLASHEx_OBGetConfig(&obConfig);

  /* If the first sector is not protected */
  if ((obConfig.WRPSector & OB_WRP_SECTOR_0) == OB_WRP_SECTOR_0) {
    HAL_FLASH_Unlock(); //Unlocks flash
    HAL_FLASH_OB_Unlock(); //Unlocks OB
    obConfig.OptionType = OPTIONBYTE_WRP;
    obConfig.WRPState = OB_WRPSTATE_ENABLE; //Enables changing of WRP settings
    obConfig.WRPSector = OB_WRP_SECTOR_0; //Enables WP on first sector
    HAL_FLASHEx_OBProgram(&obConfig); //Programs the OB
    HAL_FLASH_OB_Launch(); //Ensures that the new configuration is saved in flash
    HAL_FLASH_OB_Lock(); //Locks OB
    HAL_FLASH_Lock(); //Locks flash
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
