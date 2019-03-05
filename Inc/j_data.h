﻿/*----------------------------------------------------------------------
 *   RTOS platform data & defines
 *--------------------------------------------------------------------*/
 #pragma once

/*----------------------------------------------------------------------
 *   Common Include
 *--------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "cmsis_os.h"

/*----------------------------------------------------------------------
 *   Common Define
 *--------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 *   Common typedef
 *--------------------------------------------------------------------*/
typedef enum
{
	USB_INTERFACE	= 0 ,
	UART_INTERFACE	= 1 ,
	WIFI_INTERFACE	= 2 ,
	BT_INTERFACE	= 3
} COMM_Interface;

/*----------------------------------------------------------------------
 *   Common Structure
 *--------------------------------------------------------------------*/
typedef struct
{
	uint8_t		source;
	uint16_t	len;
	uint8_t		data[2048];
} inCommPkt_t;

typedef struct
{
	uint32_t	source;
//	uint32_t	data;
	inCommPkt_t	*p_incommpkt;
} inCommMsg_t;

typedef struct
{
	//-----------------------------------------------------------------
	//   Thread variable
	//-----------------------------------------------------------------
	osThreadId		h_sendThread;
	osThreadId		h_receiveThread;

	//-----------------------------------------------------------------
	//   Memory Pool variable
	//-----------------------------------------------------------------
	osPoolId		h_InCommPool;

	//-----------------------------------------------------------------
	//   Message variable
	//-----------------------------------------------------------------
	osMessageQId	h_InCommMessage;

} properties_t;

/*----------------------------------------------------------------------
 *   Common Function
 *--------------------------------------------------------------------*/
void sendThread( void const *argument );
void receiveThread( void const *argument );

extern properties_t	*p_properties;