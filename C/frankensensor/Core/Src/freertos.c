/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usart.h"
#include "i2c.h"
#include <string.h>
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PA1010D_ADDR (0x10 << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
  extern UART_HandleTypeDef huart3;
/* Button task handle and attributes */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ButtonTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartGPSPollingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartGPSPollingTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  buttonTaskHandle = osThreadNew(ButtonTask, NULL, &buttonTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartGPSPollingTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Recover I2C bus in case slave is stuck */
  gps_i2c_bus_recovery();

  /* Check if GPS module is present on I2C bus */
  if (HAL_I2C_IsDeviceReady(&hi2c1, PA1010D_ADDR, 3, 100) == HAL_OK) {
    HAL_UART_Transmit(&huart3, (uint8_t*)"GPS found!\r\n", /*trials*/ 12, /*timeout ms*/ 100);
  } else {
    HAL_UART_Transmit(&huart3, (uint8_t*)"GPS not found\r\n", 15, 100);
  }

  /* Infinite loop */
  for(;;)
  {
    static uint8_t gps_data[255];

    if (HAL_I2C_Master_Receive(&hi2c1, PA1010D_ADDR, gps_data, sizeof(gps_data), 100) == HAL_OK) {
      gps_data[254] = '\0';

      // Searches the buffer for the string "$GNGGA". Returns a pointer to where it found it, or NULL if it's not in the buffer. This is how we find the sentence we care about.
      char *gga = strstr((char *)gps_data, "$GNGGA");

      if (gga != NULL) {
        gps_data_t gps;

        if (nmea_parse_gga(gga, &gps) == 0) {
          char msg[96];
          int len = snprintf(msg, sizeof(msg),
            "%02d:%02d:%02d Fix:%d Sat:%d Lat:%.4f Lon:%.4f HDOP:%.1f Alt:%.0fft\r\n",
            gps.hours, gps.minutes, gps.seconds,
            gps.fix_quality, gps.satellites,
            gps.latitude, gps.longitude,
            gps.hdop, gps.altitude * 3.28084f
          );
          HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
        }
      }
    }

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  Function implementing the buttonTask thread.
  *         Waits for button press events and handles them.
  * @param  argument: Not used
  * @retval None
  */
void ButtonTask(void *argument)
{
  uint32_t flags;

  for (;;)
  {
    /* Wait indefinitely for button press event */
    flags = osThreadFlagsWait(BUTTON_PRESSED_FLAG, osFlagsWaitAny, osWaitForever);

    /* Check if we got the flag (not an error) */
    if ((flags & BUTTON_PRESSED_FLAG) != 0)
    {
      /* Button was pressed - toggle green LED as example */
      HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
  }
}
/* USER CODE END Application */

