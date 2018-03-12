/**
   MappyDot Firmware - history_buffer.h

   Copyright (C) 2017 SensorDots.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef BUFFER_H_
#define BUFFER_H_

#include <stddef.h>

typedef struct circular_history_buffer
{
    void *buffer;     // data buffer
    void *buffer_end; // end of data buffer
    size_t capacity;  // maximum number of items in the buffer
    size_t count;     // number of items in the buffer
    size_t sz;        // size of each item in the buffer
    void *head;       // pointer to head
    void *tail;       // pointer to tail
} circular_history_buffer;

void hb_init(circular_history_buffer *cb, size_t capacity, size_t sz);

void hb_free(circular_history_buffer *cb);

void hb_push_back(circular_history_buffer *cb, const void *item);

void hb_pop_front(circular_history_buffer *cb, void *item);



#endif /* BUFFER_H_ */