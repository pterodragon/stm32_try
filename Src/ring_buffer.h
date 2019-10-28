#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

// breaks encapsulation when put in header file
struct circular_buf_t {
  uint8_t *buffer;
  uint32_t head;
  uint32_t tail;
  uint32_t max;
  int8_t full;
};

// reference:
// https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc#c-implementation
typedef struct circular_buf_t circular_buf_t;

typedef circular_buf_t *cbuf_handle_t;

/// Pass in a storage buffer and size
/// Returns a circular buffer handle
// User provides struct
void circular_buf_init(circular_buf_t *cbuf, uint8_t *buffer, uint32_t size);

/// Reset the circular buffer to empty, head == tail
void circular_buf_reset(cbuf_handle_t cbuf);

/// Put version 1 continues to add data if the buffer is full
/// Old data is overwritten
void circular_buf_put(cbuf_handle_t cbuf, uint8_t data);

/// Retrieve a value from the buffer
/// Returns 0 on success, -1 if the buffer is empty
int8_t circular_buf_get(cbuf_handle_t cbuf, uint8_t *data);

/// Returns true if the buffer is empty
int8_t circular_buf_empty(cbuf_handle_t cbuf);

/// Returns true if the buffer is full
int8_t circular_buf_full(cbuf_handle_t cbuf);

/// Returns the maximum capacity of the buffer
uint32_t circular_buf_capacity(cbuf_handle_t cbuf);

/// Returns the current number of elements in the buffer
uint32_t circular_buf_size(cbuf_handle_t cbuf);

#endif /* RING_BUFFER_H */
