﻿/*----------------------------------------------------------------------
 *   RTOS platform data & defines
 *--------------------------------------------------------------------*/
#ifndef __J_DATA_H__
#define __J_DATA_H__
 
/*----------------------------------------------------------------------
 *   Common Include
 *--------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "cmsis_os.h"
#include "string.h"

#include "j_ring.h"

/*----------------------------------------------------------------------
 *   Common Define
 *--------------------------------------------------------------------*/
#define	MEMORY_POOL_SIZE							10
#define	MESSAGE_QUEUE_SIZE							10

#define PROTOCOL_MIN_SIZE							3
#define	DATA_PACKET_SIZE							2048
#define	DIAGNOSTIC_PACKET_SIZE						100
#define	OUTCOM_PACKET_SIZE							2048

#define UART_PROTOCOL_START_SIZE					3


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

typedef enum
{
	CAN_INTERFACE	= 0 ,
	KWP_INTERFACE	= 1 ,
	ETH_INTERFACE	= 2
} DIAG_Interface;

/*----------------------------------------------------------------------
 *   Common Structure
 *--------------------------------------------------------------------*/
typedef struct
{
	uint16_t	len;
	uint8_t		data[DATA_PACKET_SIZE];
} MsgPkt_t;

typedef struct
{
	uint8_t		source;
	uint16_t	mod;
	MsgPkt_t	*p_pkt;
} MsgClst_t;

typedef struct
{
	//-----------------------------------------------------------------
	//   Status Check variable
	//-----------------------------------------------------------------
	uint8_t			m_uartInFlag;

	//-----------------------------------------------------------------
	//   Memory Pool variable
	//-----------------------------------------------------------------
	osPoolId		h_InCommPool;
//	osPoolId		h_DiagPool;
	osPoolId		h_OutCommPool;

	//-----------------------------------------------------------------
	//   Message variable
	//-----------------------------------------------------------------
	osMessageQId	h_InCommMessage;
	osMessageQId	h_CanCommMessage;
	osMessageQId	h_KwpCommMessage;
	osMessageQId	h_EthCommMessage;
	osMessageQId	h_OutCommMessage;
	
	//-----------------------------------------------------------------
	//   Thread variable
	//-----------------------------------------------------------------
	osThreadId		h_uartRecThread;
	osThreadId		h_receiveThread;

	osThreadId		h_canDiagThread;
	osThreadId		h_kwpDiagThread;
	osThreadId		h_ethDiagThread;

	osThreadId		h_sendThread;
} properties_t;

/*----------------------------------------------------------------------
 *   Common Function
 *--------------------------------------------------------------------*/
void sendThread( void const *argument );
void uartRecThread( void const *argument );
void receiveThread( void const *argument );
void canDiagThread( void const *argument );
void kwpDiagThread( void const *argument );
void ethDiagThread( void const *argument );

extern properties_t	*p_properties;

#endif // __J_DATA_H__
