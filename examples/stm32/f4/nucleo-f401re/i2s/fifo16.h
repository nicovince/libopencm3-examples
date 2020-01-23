#ifndef __FIFO16_H__
#define __FIFO16_H__
#include <stdint.h>
#include <stdlib.h>

typedef struct {
	size_t size;
	size_t head;
	size_t tail;
	uint16_t *array;
} fifo16_t;

void init_fifo16(fifo16_t * fifo_ptr, uint16_t *array, size_t size);
void push16(fifo16_t *fifo_ptr, uint16_t data);
uint16_t pop16(fifo16_t *fifo_ptr);
uint8_t is_full16(fifo16_t * fifo_ptr);
uint8_t is_empty16(fifo16_t * fifo_ptr);
size_t size_available16(fifo16_t * fifo_ptr);
size_t fifo_level16(fifo16_t * fifo_ptr);

#endif /* __FIFO16_H__ */

