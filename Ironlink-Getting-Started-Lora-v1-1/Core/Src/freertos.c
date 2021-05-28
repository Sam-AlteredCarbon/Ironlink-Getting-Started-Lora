/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "ironlink-library.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN StartDefaultTask */
	//Initialise Lora modem congfiuration options port and buffer
	il_lora_config lora_config;
	char buffer[50] = {'\0'};
	uint8_t port = 0x01;
	// Init code for USB_DEVICE
	il_usb_init();
	//Set the start of line character for the usb
	il_usb_set_sol_char(&usb, '>');
	 //Set the start of line character for the usb
	il_usb_set_eol_char(&usb, '\r');

	//Send message to make the USB peripheral discoverable by the PC
	il_printf("Enable Transmission\r\n");
	il_delay(1000);
	il_printf("Begin Transmission\r\n");
	il_delay(4000);


	// Enable the modem
	// Either RN2483 or RN2903 depending on modem model number
	il_lora_enable_modem(RN2483);

	// Setup Lorawan device and network configuration.
	il_lora_modem_default_config(&lora_config, RN2483);

	lora_config.appEui 				= "0000000000000000";					// AppEui can be left as all 0s or you can make customize your own. Must be 16 characters long.
	lora_config.appKey 				= "00000000000000000000000000000000";   // AppKey is generated on the things network when you create a device. Input the 32 character key here.

	// Initisalise the modem
	il_lora_modem_init(&lora_config);

	// Check if user is ready to join the network
	il_printf("\r\n Join network (y/n)? \r\n");

	il_usb_read_line(&usb, buffer);
	// If yes join the network
	if(strstr((char*)buffer, "y") != NULL)
	{
		il_lora_modem_join_network(&lora_config);
	}
	// Clear the buffer.
	il_clear_buffer(buffer, 50);


	/* Infinite loop */
	for(;;)
	{
	  // Check if user is ready to send a data packet
	  il_printf("\r\n Send Packet (y/n)? \r\n");
	  il_usb_read_line(&usb, buffer);
	  if(strstr((char*)buffer, "y") != NULL)
	  {
		  il_lora_status status = il_lora_modem_send_packet_u32(0xFF, port, RN_MAC_TX_UNCONFIRMED);

		  if(status == IL_LORA_MAC_TX_OK) {
			  il_delay(10);
			  il_printf("Transmit Good!\r\n");
		  }
	  }
	  il_clear_buffer(buffer, 50);

	  il_printf("Test Working\r\n");

	  il_delay(90000);

	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
