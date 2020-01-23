#ifndef __FIFO_H__
#define __FIFO_H__
#include <stdint.h>
#include <stdlib.h>

typedef struct {
	size_t size;
	size_t head;
	size_t tail;
	uint8_t *array;
} fifo_t;

void init_fifo(fifo_t * fifo_ptr, uint8_t *array, size_t size);
void push(fifo_t *fifo_ptr, uint8_t data);
void push16(fifo_t *fifo_ptr, uint16_t data);
uint8_t pop(fifo_t *fifo_ptr);
uint16_t pop16(fifo_t *fifo_ptr);
uint8_t is_full(fifo_t * fifo_ptr);
uint8_t is_empty(fifo_t * fifo_ptr);
/* Empty space available */
size_t size_available(fifo_t * fifo_ptr);
size_t size_available16(fifo_t * fifo_ptr);
/* Used space */
size_t fifo_level(fifo_t * fifo_ptr);
size_t fifo_level16(fifo_t * fifo_ptr);
void flush(fifo_t *fifo_ptr);

#endif // __FIFO_H__

