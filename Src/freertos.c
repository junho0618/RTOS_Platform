/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "j_data.h"

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
uint8_t	g_cUserBtnFlag;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	for(;;)
	{
		if( g_cUserBtnFlag == 1 )
		{
			g_cUserBtnFlag = 0;

//			HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );
//			HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
//			HAL_GPIO_TogglePin( LD3_GPIO_Port, LD3_Pin );
		}
		
		HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );

		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void sendThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		osDelay( 10 );
	}
}

void receiveThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	inCommMsg_t		*message;
	osEvent	evt;

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt = osMessageGet( ivp->h_InCommMessage, osWaitForever );

		message	= (inCommMsg_t * )evt.value.p;
				
#if 0	// for test				
		message->p_incommpkt->data[message->p_incommpkt->len] = NULL;
//		printf( "i = %d\r\n", message->index );
//		printf( "s = %d\r\n", message->p_incommpkt->source );
//		printf( "l = %d\r\n", message->p_incommpkt->len );
		printf( "d = %s\r", message->p_incommpkt->data );
#endif		
		if( ( message->p_pkt->data[0] == 's' ) &&
			( message->p_pkt->data[1] == 't' ) &&
			( message->p_pkt->data[2] == 'a' ) &&
			( message->p_pkt->data[3] == 'r' ) &&
			( message->p_pkt->data[4] == 't' ) )
		{
			HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
			
			message->p_pkt->data[message->p_pkt->len] = NULL;				// for test printf
			printf( "d = %s\r", message->p_pkt->data );
		}
		

		osPoolFree( ivp->h_InCommPool, message->p_pkt );

		osDelay( 100 );
	}
}

void canDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		osDelay( 10 );
	}
}

void kwpDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		osDelay( 10 );
	}
}

void ethDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		osDelay( 10 );
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
