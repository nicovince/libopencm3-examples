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

void push16(fifo_t *fifo_ptr, uint16_t data)
{
	push(fifo_ptr, (uint8_t) data & 0xFF);
	push(fifo_ptr, (uint8_t) (data >> 8) & 0xFF);
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

uint16_t pop16(fifo_t *fifo_ptr)
{
	uint16_t data;

	data = pop(fifo_ptr);
	data |= pop(fifo_ptr) << 8;
	return data;
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


void flush(fifo_t *fifo_ptr)
{
	fifo_ptr->head = 0;
	fifo_ptr->tail = 0;
}

