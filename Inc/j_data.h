/*----------------------------------------------------------------------
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
#define	MEMORY_POOL_SIZE							10
#define	MESSAGE_QUEUE_SIZE							10

#define	INCOM_PACKET_SIZE							2048
#define	DIAGNOSTIC_PACKET_SIZE						100
#define	OUTCOM_PACKET_SIZE							2048


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
// InComm
typedef struct
{
	uint8_t		source;
	uint16_t	len;
	uint8_t		data[INCOM_PACKET_SIZE];
} inCommPkt_t;

typedef struct
{
	uint32_t	source;
	inCommPkt_t	*p_pkt;
} inCommMsg_t;

// Diagnostic
typedef struct
{
	uint8_t		categori;
	uint16_t	len;
	uint8_t		data[DIAGNOSTIC_PACKET_SIZE];	
} diagPkt_t;

typedef struct
{
	uint8_t		source;
	diagPkt_t	*p_pkt;	
} diagMsg_t;

// OutComm
typedef struct
{
	uint8_t		source;
	uint16_t	len;
	uint8_t		data[OUTCOM_PACKET_SIZE];
} outCommPkt_t;

typedef struct
{
	uint32_t		source;
	outCommPkt_t	*p_pkt;
} outCommMsg_t;

typedef struct
{
	//-----------------------------------------------------------------
	//   Thread variable
	//-----------------------------------------------------------------
	osThreadId		h_sendThread;
	osThreadId		h_receiveThread;
	osThreadId		h_canDiagThread;
	osThreadId		h_kwpDiagThread;
	osThreadId		h_ethDiagThread;

	//-----------------------------------------------------------------
	//   Memory Pool variable
	//-----------------------------------------------------------------
	osPoolId		h_InCommPool;
	osPoolId		h_DiagPool;
	osPoolId		h_OutCommPool;

	//-----------------------------------------------------------------
	//   Message variable
	//-----------------------------------------------------------------
	osMessageQId	h_InCommMessage;
	osMessageQId	h_CanCommMessage;
	osMessageQId	h_KwpCommMessage;
	osMessageQId	h_EthCommMessage;
	osMessageQId	h_OutCommMessage;
} properties_t;

/*----------------------------------------------------------------------
 *   Common Function
 *--------------------------------------------------------------------*/
void sendThread( void const *argument );
void receiveThread( void const *argument );
void canDiagThread( void const *argument );
void kwpDiagThread( void const *argument );
void ethDiagThread( void const *argument );


extern properties_t	*p_properties;