/**
 * \file ring_buffer.c
 * \author Chris Karaplis
 * \brief Ring buffer implementation
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

#include "RingBuffer.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

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
int ring_buffer_init(rbd_t *rbd, rb_attr_t *attr)
{
    static int idx = 0;
    int err = -1;

    if ((idx < RING_BUFFER_MAX) && (rbd != NULL) && (attr != NULL)) {
        if ((attr->buffer != NULL) && (attr->s_elem > 0)) {
            /* Check that the size of the ring buffer is a power of 2 */
            if (((attr->n_elem - 1) & attr->n_elem) == 0) {
                /* Initialize the ring buffer internal variables */
                _rb[idx].head = 0;
                _rb[idx].tail = 0;
                _rb[idx].buf = attr->buffer;
                _rb[idx].s_elem = attr->s_elem;
                _rb[idx].n_elem = attr->n_elem;

                *rbd = idx++;
                err= 0;
            }
        }
    }

    return err;
}

/**
 * \brief Add an element to the ring buffer
 * \param[in] rbd - the ring buffer descriptor
 * \param[in] data - the data to add
 * \return 0 on success, -1 otherwise
 */
int ring_buffer_put(rbd_t rbd, const void *data)
{
    int err = 0;

    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_full(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].head & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem; //this is to avoid direct modular arithmetic
        memcpy(&(_rb[rbd].buf[offset]), data, _rb[rbd].s_elem);
        _rb[rbd].head++;
    } else {
        err = -1;
    }

    return err;
}

/**
 * \brief Get (and remove) an element from the ring buffer
 * \param[in] rb - the ring buffer descriptor
 * \param[in] data - pointer to store the data
 * \return 0 on success, -1 otherwise
 */
int ring_buffer_get(rbd_t rbd, void *data)
{
    int err = 0;

    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_empty(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].tail & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem; //this is to aovid direct modular arithmetic
        memcpy(data, &(_rb[rbd].buf[offset]), _rb[rbd].s_elem);
        _rb[rbd].tail++; //tail is only incremented after copy to prevent the necessity of a critical section
    } else {
        err = -1;
    }

    return err;
}

// Helper function to locate a given string within the ring buffer
// returns - number of elements to 'get' from the string to remove the given string from the ring buffer

int ring_buffer_strfind(rbd_t rbd, const char *str){
    size_t partialMatch = 0;
    size_t tail;
    for(tail = _rb[rbd].tail;tail!=_rb[rbd].head;tail++){
        //head is guaranteed to always be greater than tail as they're both 'unwound' indices
        //...the condition could be improved, but considering the length of the const char array
        //...is unknown at runtime it's simpler this way

        size_t offset = (tail & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem; //calculate the offset to the tail
                                                                                    //from 'unwound tail and head'
        char c = *str;

        if(_rb[rbd].buf[offset]==c){
            partialMatch++;
            str++;
            if(*str=='\0'){
                return (tail -_rb[rbd].tail) + 1; // if we get to the end of the search string, we've found it, return the num of elements to take out of the buffer
            }
        }else{
            str-=partialMatch; //reset the search string to original pointer
            tail-=partialMatch; //reset the tail to the length that we've looked... will be incremented by one before next loop iteration
            partialMatch = 0; //reset partialMatch counter
        }

    }
    return 0; //string not contained, return the number of elements to remove from the buffer (zero in this case)
}


void ring_buffer_clear(rbd_t rbd){
    _rb[rbd].tail=_rb[rbd].head; //set tail to head, effectively clear the buffer
}

size_t ring_buffer_size(rbd_t rbd){
    return _rb[rbd].head - _rb[rbd].tail;
}


static int is_ring_buffer_full(rbd_t rbd){
    return ((_rb[rbd].head - _rb[rbd].tail)  == _rb[rbd].n_elem) ? 1 : 0;
}
