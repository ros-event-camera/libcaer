/*
 * ringbuffer.h
 *
 *  Created on: Dec 10, 2013
 *      Author: llongi
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#ifdef __cplusplus

#include <cstdlib>
#include <cstdint>

#else

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ring_buffer *RingBuffer;

RingBuffer ringBufferInit(size_t size);
void ringBufferFree(RingBuffer rBuf);
bool ringBufferPut(RingBuffer rBuf, void *elem);
bool ringBufferFull(RingBuffer rBuf);
void *ringBufferGet(RingBuffer rBuf);
void *ringBufferLook(RingBuffer rBuf);

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H_ */
