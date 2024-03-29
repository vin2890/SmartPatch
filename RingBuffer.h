/*
 * ring_buffer.h
 *
 *  Created on: Feb 27, 2019
 *      Author: Kyle Green
 */
/**
 * \file ring_buffer.h
 * \author Chris Karaplis
 * \brief Ring buffer API
 *
 * Copyright (c) 2015, simplyembedded.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>
#include <stddef.h>

/* Ring buffer descriptor */
typedef unsigned int rbd_t;

/* User defined ring buffer attributes */
typedef struct {
    size_t s_elem;
    size_t n_elem;
    void *buffer;
} rb_attr_t;

#define RING_BUFFER_MAX  2 //two ring buffers allowed

struct ring_buffer
{
    size_t s_elem;
    size_t n_elem;
    uint8_t *buf;
    volatile size_t head;
    volatile size_t tail;
};

static struct ring_buffer _rb[RING_BUFFER_MAX];


/**
 * \brief Initialize a ring buffer
 * \param[out] rb - pointer to a ring buffer descriptor
 * \param[in] attr - ring buffer attributes
 * \return 0 on success, -1 otherwise
 *
 * The attributes must contain a ring buffer which is sized
 * to an even power of 2. This should be reflected by the
 * attribute n_elem.
 */
int ring_buffer_init(rbd_t *rbd, rb_attr_t *attr);

/**
 * \brief Add an element to the ring buffer
 * \param[in] rb - the ring buffer descriptor
 * \param[in] data - the data to add
 * \return 0 on success, -1 otherwise
 */
int ring_buffer_put(rbd_t rbd, const void *data);

/**
 * \brief Get (and remove) an element from the ring buffer
 * \param[in] rb - the ring buffer descriptor
 * \param[in] data - pointer to store the data
 * \return 0 on success, -1 otherwise
 */
int ring_buffer_get(rbd_t rbd, void *data);

int ring_buffer_strfind(rbd_t rbd, const char *str);

void ring_buffer_clear(rbd_t rbd);

size_t ring_buffer_size(rbd_t rbd);

static int _ring_buffer_full(struct ring_buffer *rb);

static int _ring_buffer_empty(struct ring_buffer *rb);

static int is_ring_buffer_full(rbd_t rbd);


static int _ring_buffer_full(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == rb->n_elem) ? 1 : 0;
}

static int _ring_buffer_empty(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == 0U) ? 1 : 0;
}
#endif /* RING_BUFFER_H_ */
