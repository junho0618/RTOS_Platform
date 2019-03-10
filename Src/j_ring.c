#include "j_ring.h"

void InitRingBuffer( RingBuffer_t *queue )
{
    queue->front	= 0;
    queue->rear		= 0;
    queue->count	= 0;
}

int IsFull( RingBuffer_t *queue )
{
    return queue->count == QUEUE_SIZE;
}

int IsEmpty( RingBuffer_t *queue )
{
    return queue->count == 0;
}

void Enqueue( RingBuffer_t *queue, int data )
{
    if( IsFull(queue) )
    {
        printf("Queue is Full\r\n");
        return;
    }

    queue->buf[queue->rear]	= data;
    queue->rear				= NEXT(queue->rear);
    queue->count++;
}

int Dequeue( RingBuffer_t *queue )
{
    int re = 0;
    
    if( IsEmpty(queue) )
    {
        printf("Queue is Empty\r\n");
        return re;
    }
    
    re				= queue->buf[queue->front];
    queue->front	= NEXT(queue->front);
    queue->count--;
    
    return re;
}

int GetQueueSize( RingBuffer_t *queue )
{	
	return queue->count;
}

void ClearQueue( RingBuffer_t *queue )
{
	queue->front	= 0;
    queue->rear		= 0;
    queue->count	= 0;
}