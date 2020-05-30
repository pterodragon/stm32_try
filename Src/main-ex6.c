/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <cmsis_os.h>
#include <nucleo_hal_bsp.h>
#include <string.h>
#include "stm32f4xx.h"

void blinkThread(void const *argument);

uint8_t suspendBlink = 0;
osThreadId blinkTID;
osMessageQId GPIOEvent = 0;

extern volatile uint8_t ucSleepModeCalled;

static volatile uint32_t sleepTime = 50;

void Hard_Fault_Handler(uint32_t stack[]);

void printUsageErrorMsg(uint32_t CFSRValue)
{
   printf("Usage fault: ");
   CFSRValue >>= 16;                  // right shift to lsb
   if((CFSRValue & (1 << 9)) != 0) {
      printf("Divide by zero\n");
   }
}


enum { r0, r1, r2, r3, r12, lr, pc, psr};

void Hard_Fault_Handler(uint32_t stack[])
{
   static char msg[80];
   printf("In Hard Fault Handler\n");
   sprintf(msg, "SCB->HFSR = 0x%08lx\n", SCB->HFSR);
   printf(msg);
   if ((SCB->HFSR & (1 << 30)) != 0) {
       printf("Forced Hard Fault\n");
       sprintf(msg, "SCB->CFSR = 0x%08lx\n", SCB->CFSR );
       printf(msg);
       if((SCB->CFSR & 0xFFFF0000) != 0) {
         printUsageErrorMsg(SCB->CFSR);
      }
   }

   sprintf(msg, "r0  = 0x%08lx\n", stack[r0]);  printf(msg);
   sprintf(msg, "r1  = 0x%08lx\n", stack[r1]);  printf(msg);
   sprintf(msg, "r2  = 0x%08lx\n", stack[r2]);  printf(msg);
   sprintf(msg, "r3  = 0x%08lx\n", stack[r3]);  printf(msg);
   sprintf(msg, "r12 = 0x%08lx\n", stack[r12]); printf(msg);
   sprintf(msg, "lr  = 0x%08lx\n", stack[lr]);  printf(msg);
   sprintf(msg, "pc  = 0x%08lx\n", stack[pc]);  printf(msg);
   sprintf(msg, "psr = 0x%08lx\n", stack[psr]); printf(msg);
   __ASM volatile("BKPT #01");
   while(1);
}

int main(void) {
  HAL_DBGMCU_EnableDBGSleepMode();
  HAL_DBGMCU_EnableDBGStopMode();

  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
  SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
  uint32_t sr = &SCB->CFSR;

  HAL_Init();
  Nucleo_BSP_Init();

  osThreadDef(blink, blinkThread, osPriorityNormal, 0, 1000);
  blinkTID = osThreadCreate(osThread(blink), NULL);

  osMessageQDef(GPIOQueue, 1, uint8_t);
  GPIOEvent = osMessageCreate(osMessageQ(GPIOQueue), NULL);

  osKernelStart();

  /* Infinite loop */
  while (1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == B1_Pin) {
    osMessagePut(GPIOEvent, 1, 0);
    if (suspendBlink)
      osThreadResume(blinkTID);
  }
}


void blinkThread(void const *argument) {
  UNUSED(argument);

  osEvent evt;

  while (1) {
    evt = osMessageGet(GPIOEvent, 0);
    if (evt.status == osEventMessage) {
      ucSleepModeCalled = 0;
      if (suspendBlink) {
        suspendBlink = 0;
        sleepTime = 50;
      } else if (sleepTime == 50) {
        sleepTime = 500;
      } else if (sleepTime == 500) {
        suspendBlink = 1;
        osThreadSuspend(NULL);
      }
    }
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(sleepTime);
  }
}

#if configUSE_TICKLESS_IDLE == 2

void preSLEEP(TickType_t xModifiableIdleTime) {
  UNUSED(xModifiableIdleTime);

  HAL_SuspendTick();
  __enable_irq();
  __disable_irq();
}

void postSLEEP(TickType_t xModifiableIdleTime) {
  UNUSED(xModifiableIdleTime);

  HAL_ResumeTick();
}

void preSTOP() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  __HAL_RCC_GPIOA_CLK_DISABLE();
  HAL_SuspendTick();
  __enable_irq();
  __disable_irq();
}

void postSTOP() {
  SystemClock_Config();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_ResumeTick();
}


#endif

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
  UNUSED(pxTask);
  UNUSED(pcTaskName);
  asm("BKPT #0");
}

#endif

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

