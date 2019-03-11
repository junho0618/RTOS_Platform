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
#include "j_ring.h"

#include "usart.h"
#include "usbd_cdc_if.h"

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
		
//		HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );

		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void sendThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	osEvent			evt;
	MsgClst_t		*message;	

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt		= osMessageGet( ivp->h_OutCommMessage, osWaitForever );
		message	= ( MsgClst_t * )evt.value.p;
		
		// add Send Output Interface code 
		
		switch( message->source )
		{
			case USB_INTERFACE	:	
				CDC_Transmit_FS( message->p_pkt->data, message->p_pkt->len );
//				message->p_pkt->data[message->p_pkt->len] = NULL;				// for test printf
//				printf( "%s\r\n", message->p_pkt->data );			

				break;
				
			case UART_INTERFACE	:
				HAL_UART_Transmit( &huart2, message->p_pkt->data, message->p_pkt->len, 300 );
				break;
				
			case WIFI_INTERFACE	:
				break;
				
			case BT_INTERFACE	:				
				break;
				
			default :
				break;
		}
		
		osPoolFree( ivp->h_OutCommPool, message->p_pkt );
		
		osDelay( 100 );
	}
}

void receiveThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	osEvent			evt;
	MsgClst_t		*message;	
	
//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt		= osMessageGet( ivp->h_InCommMessage, osWaitForever );
		message	= ( MsgClst_t * )evt.value.p;
				
#if 0	// for test				
		message->p_incommpkt->data[message->p_incommpkt->len] = NULL;
//		printf( "i = %d\r\n", message->index );
//		printf( "s = %d\r\n", message->p_incommpkt->source );
//		printf( "l = %d\r\n", message->p_incommpkt->len );
		printf( "d = %s\r", message->p_incommpkt->data );
#endif		
		if( ( message->p_pkt->data[0] == 'G' ) &&
			( message->p_pkt->data[1] == 'I' ) &&
			( message->p_pkt->data[2] == 'T' ) )
		{
			MsgClst_t		target_message;
			osMessageQId	h_target;
			
			int32_t			target	= message->p_pkt->data[3] % 3;
			
			switch( target )
			{
				case CAN_INTERFACE :
					h_target	= ivp->h_CanCommMessage;
					break;
					
				case KWP_INTERFACE :
					h_target	= ivp->h_KwpCommMessage;
					break;
					
				case ETH_INTERFACE :
					h_target	= ivp->h_EthCommMessage;
					break;
					
				default	:
					break;
			}
			
			target_message.source	= message->source;
			target_message.p_pkt	= message->p_pkt;
			osMessagePut( h_target, (uint32_t)&target_message, osWaitForever );	
		}
		
//		message->p_pkt->data[message->p_pkt->len] = NULL;				// for test printf
//		printf( "%s\r", message->p_pkt->data );			

//		osPoolFree( ivp->h_InCommPool, message->p_pkt );
		
//		HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );

		osDelay( 100 );
	}
}

void canDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	osEvent			evt;
	MsgClst_t		*message;
	
	MsgPkt_t		*packet;
	MsgClst_t		target_message;
	
	uint8_t			test_string[30];

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt		= osMessageGet( ivp->h_CanCommMessage, osWaitForever );
		message	= ( MsgClst_t * )evt.value.p;
		
		// add Parsign & Diagnostic code
		
#if 1	// test		
		sprintf( (char *)test_string, "Can processed Date!!" );

		packet		= ( MsgPkt_t * )osPoolAlloc( ivp->h_OutCommPool );
		packet->len	= strlen( (char *)test_string );
		memcpy( (void *)packet->data, (const void *)test_string, strlen( (char *)test_string ) );
		
		target_message.source	= message->source;
		target_message.p_pkt	= packet;
		osMessagePut( ivp->h_OutCommMessage, (uint32_t)&target_message, osWaitForever );
#endif		
		
		osPoolFree( ivp->h_InCommPool, message->p_pkt );
		
		HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );
		
		osDelay( 100 );
	}
}

void kwpDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	osEvent			evt;
	MsgClst_t		*message;	

	MsgPkt_t		*packet;
	MsgClst_t		target_message;
	
	uint8_t			test_string[30];

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt		= osMessageGet( ivp->h_KwpCommMessage, osWaitForever );
		message	= ( MsgClst_t * )evt.value.p;
		
		// add Parsign & Diagnostic code 
		
#if 1	// test		
		sprintf( (char *)test_string, "KWP processed Date!!" );

		packet		= ( MsgPkt_t * )osPoolAlloc( ivp->h_OutCommPool );
		packet->len	= strlen( (char *)test_string );
		memcpy( (void *)packet->data, (const void *)test_string, strlen( (char *)test_string ) );
		
		target_message.source	= message->source;
		target_message.p_pkt	= packet;
		osMessagePut( ivp->h_OutCommMessage, (uint32_t)&target_message, osWaitForever );
#endif
		
		osPoolFree( ivp->h_InCommPool, message->p_pkt );
		
		HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
		
		osDelay( 100 );
	}
}

void ethDiagThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	osEvent			evt;
	MsgClst_t		*message;	

	MsgPkt_t		*packet;
	MsgClst_t		target_message;
	
	uint8_t			test_string[30];

//	printf( "%s\r\n", __func__ );
	for(;;)
	{
		evt		= osMessageGet( ivp->h_EthCommMessage, osWaitForever );
		message	= ( MsgClst_t * )evt.value.p;
		
		// add Parsign & Diagnostic code 
		
#if 1	// test		
		sprintf( (char *)test_string, "ETH processed Date!!" );

		packet		= ( MsgPkt_t * )osPoolAlloc( ivp->h_OutCommPool );
		packet->len	= strlen( (char *)test_string );
		memcpy( (void *)packet->data, (const void *)test_string, strlen( (char *)test_string ) );
		
		target_message.source	= message->source;
		target_message.p_pkt	= packet;
		osMessagePut( ivp->h_OutCommMessage, (uint32_t)&target_message, osWaitForever );
#endif
		
		osPoolFree( ivp->h_InCommPool, message->p_pkt );
		
		HAL_GPIO_TogglePin( LD3_GPIO_Port, LD3_Pin );
		
		osDelay( 100 );
	}
}

void uartRecThread( void const *argument )
{
	properties_t	*ivp	= (properties_t *)argument;
	
	MsgPkt_t	*packet;
	MsgClst_t	message;
	
	uint16_t	len;
	uint16_t	i;
	uint16_t	waitTime;

	
	for(;;)
	{
		/* Uart Data가 들어올때까지 Wait */
		while( !ivp->m_uartInFlag )
		{
			osDelay( 100 );
		}
		
		if( GetQueueSize( &rbUartRx ) >= UART_PROTOCOL_START_SIZE )
		{
			if( Dequeue( &rbUartRx ) == 'G' )
			{
				if( Dequeue( &rbUartRx ) == 'I' )
				{
					if( Dequeue( &rbUartRx ) == 'T' )
					{					
						/* Get Data Length */
						len = Dequeue( &rbUartRx ) * 256;
						len = len + Dequeue( &rbUartRx );
						printf( "data len = %d\r\n", len );
							
						/* 모든 데이터가 수신될때까지 대기 */
						waitTime = 300;		// 3s
						while( GetQueueSize( &rbUartRx ) < len )
						{							
							waitTime--;
							if( waitTime == 0 )
							{
								printf( "error... timeout receive uart data!!\r\n" );
								while( !IsEmpty( &rbUartRx ) )
								{
									Dequeue( &rbUartRx );
								}
								
								break;
							}
							
							osDelay( 10 );
						}
						
						if( waitTime == 0 )		continue;						// uart 처리 Reset
							
						/* 수신된 데이트를 Packet에 저장 */
						packet	= ( MsgPkt_t * )osPoolAlloc( ivp->h_InCommPool );
						packet->len	= len;						
							
						for( i = 0; i < len; i++ )
						{						
							packet->data[i] = Dequeue( &rbUartRx );
							printf( "0x%02x\r\n", packet->data[i] );										
						}
							
						/* Packet을 InCommMessage에 전달 */
						message.source	= UART_INTERFACE;
						message.p_pkt	= packet;
						osMessagePut( ivp->h_InCommMessage, (uint32_t)&message, osWaitForever );
			
						ivp->m_uartInFlag	= 0;									
					}
				}
			}			
		}
		
		osDelay( 10 );
	}	
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
