/*----------------------------------------------------------------------
 *   Ring Buffer
 *--------------------------------------------------------------------*/
#ifndef __J_RING_H__
#define __J_RING_H__
 
#include "stm32h7xx_hal.h"
//#include "cmsis_os.h"
//#include "string.h"

#define	QUEUE_SIZE					512
#define	NEXT(index)					((index+1)%QUEUE_SIZE)

typedef struct
{
	uint8_t             buf[QUEUE_SIZE];
	uint8_t             count;
	uint8_t             dummy;	
	volatile uint8_t    rxd;
	volatile uint16_t   front;
	volatile uint16_t   rear;
} RingBuffer_t;

void	InitRingBuffer( RingBuffer_t *queue );
int		IsFull( RingBuffer_t *queue );
int		IsEmpty( RingBuffer_t *queue );
void	Enqueue( RingBuffer_t *queue, int data );
int		Dequeue( RingBuffer_t *queue );
int		GetQueueSize( RingBuffer_t *queue );
void	ClearQueue( RingBuffer_t *queue );

extern RingBuffer_t		rbUartRx;
#endif // __J_DATA_H__