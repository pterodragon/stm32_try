#include "ring_buffer.h"
#include <assert.h>

void circular_buf_init(circular_buf_t* cbuf, uint8_t* buffer, uint32_t size) {
  assert(buffer && size);
  assert(cbuf);

  cbuf->buffer = buffer;
  cbuf->max = size;
  circular_buf_reset(cbuf);

  assert(circular_buf_empty(cbuf));
}

void circular_buf_reset(cbuf_handle_t cbuf) {
  assert(cbuf);

  cbuf->head = 0;
  cbuf->tail = 0;
  cbuf->full = 0;
}

int8_t circular_buf_full(cbuf_handle_t cbuf) {
  assert(cbuf);

  return cbuf->full;
}

int8_t circular_buf_empty(cbuf_handle_t cbuf) {
  assert(cbuf);

  return (!cbuf->full && (cbuf->head == cbuf->tail));
}

uint32_t circular_buf_capacity(cbuf_handle_t cbuf) {
  assert(cbuf);

  return cbuf->max;
}

uint32_t circular_buf_size(cbuf_handle_t cbuf) {
  assert(cbuf);

  uint32_t size = cbuf->max;

  if (!cbuf->full) {
    if (cbuf->head >= cbuf->tail) {
      size = (cbuf->head - cbuf->tail);
    } else {
      size = (cbuf->max + cbuf->head - cbuf->tail);
    }
  }

  return size;
}

static void advance_pointer(cbuf_handle_t cbuf) {
  assert(cbuf);

  if (cbuf->full) {
    cbuf->tail = (cbuf->tail + 1) % cbuf->max;
  }

  cbuf->head = (cbuf->head + 1) % cbuf->max;
  cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(cbuf_handle_t cbuf) {
  assert(cbuf);

  cbuf->full = 0;
  cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

// overwrite oldest item when buffer is full
void circular_buf_put(cbuf_handle_t cbuf, uint8_t data) {
  assert(cbuf && cbuf->buffer);

  cbuf->buffer[cbuf->head] = data;

  advance_pointer(cbuf);
}

int8_t circular_buf_get(cbuf_handle_t cbuf, uint8_t *data) {
  assert(cbuf && data && cbuf->buffer);

  int8_t r = -1;

  if (!circular_buf_empty(cbuf)) {
    *data = cbuf->buffer[cbuf->tail];
    retreat_pointer(cbuf);

    r = 0;
  }

  return r;
}
