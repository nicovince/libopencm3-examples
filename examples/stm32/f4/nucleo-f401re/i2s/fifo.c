#include "fifo.h"
#include <string.h>

void init_fifo(fifo_t * fifo_ptr, uint8_t *array, size_t size)
{
	// Clear struct
	memset(fifo_ptr, 0, sizeof(fifo_t));
	// Clear data array
	memset(array, 0, size);
	fifo_ptr->array = array;
	fifo_ptr->size = size;
}

void push(fifo_t *fifo_ptr, uint8_t data)
{
	size_t next = (fifo_ptr->head + 1) % fifo_ptr->size;

	if (next != fifo_ptr->tail)
	{
		fifo_ptr->array[fifo_ptr->head] = data;
		fifo_ptr->head = next;
	}
}

uint8_t pop(fifo_t *fifo_ptr)
{
	uint8_t ret = 0;
	if (fifo_ptr->head != fifo_ptr->tail)
	{
		ret = fifo_ptr->array[fifo_ptr->tail];
		fifo_ptr->tail = (fifo_ptr->tail + 1) % fifo_ptr->size;
	}
	return ret;
}

uint8_t is_full(fifo_t * fifo_ptr)
{
	return (((fifo_ptr->head + 1) % fifo_ptr->size) == fifo_ptr->tail);
}

uint8_t is_empty(fifo_t * fifo_ptr)
{
	return (fifo_ptr->head == fifo_ptr->tail);
}

size_t size_available(fifo_t * fifo_ptr)
{
	if (fifo_ptr->head >= fifo_ptr->tail) {
		return fifo_ptr->size - fifo_ptr->head + fifo_ptr->tail;
	} else {
		return fifo_ptr->tail - fifo_ptr->head;
	}
}

size_t fifo_level(fifo_t * fifo_ptr)
{
	return fifo_ptr->size - size_available(fifo_ptr);
}

void flush(fifo_t *fifo_ptr)
{
	fifo_ptr->head = 0;
	fifo_ptr->tail = 0;
}

