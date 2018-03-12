/*
 * buffer.c
 *
 * Created: 21/07/2017 12:01:24 AM
 *  Author: https://stackoverflow.com/questions/827691/how-do-you-implement-a-circular-buffer-in-c
 */

#include "history_buffer.h"

#include <stdlib.h>
#include <string.h>


void hb_init(circular_history_buffer *hb, size_t capacity, size_t sz)
{
    hb->buffer = malloc(capacity * sz);
    //if(cb->buffer == NULL)
    // handle error
    hb->buffer_end = (char *)hb->buffer + capacity * sz;
    hb->capacity = capacity;
    hb->count = 0;
    hb->sz = sz;
    hb->head = hb->buffer;
    hb->tail = hb->buffer;
}

void hb_free(circular_history_buffer *cb)
{
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

void hb_push_back(circular_history_buffer *hb, const void *item)
{
    if(hb->count == hb->capacity)
    {
        //"Pop" then continue
        hb->tail = (char*)hb->tail + hb->sz;

        if(hb->tail == hb->buffer_end)
            hb->tail = hb->buffer;

        hb->count--;
    }

    memcpy(hb->head, item, hb->sz);
    hb->head = (char*)hb->head + hb->sz;

    if(hb->head == hb->buffer_end)
        hb->head = hb->buffer;

    hb->count++;
}

void hb_pop_front(circular_history_buffer *hb, void *item)
{
    if(hb->count == 0)
    {
        // do nothing if empty
    }
    else
    {
        memcpy(item, hb->tail, hb->sz);
        hb->tail = (char*)hb->tail + hb->sz;

        if(hb->tail == hb->buffer_end)
            hb->tail = hb->buffer;

        hb->count--;
    }
}