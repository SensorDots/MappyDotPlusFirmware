/**
   MappyDot Firmware - bwlpf.h

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


#ifndef BWLPF_H_
#define BWLPF_H_


#define FILTER_ORDER                  4  //Filter order

typedef struct
{
    int n;
    double s;
    double f;
    double a;
    double a2;
    double r;
    double A[FILTER_ORDER/2];
    double d1[FILTER_ORDER/2];
    double d2[FILTER_ORDER/2];
    double w0[FILTER_ORDER/2];
    double w1[FILTER_ORDER/2];
    double w2[FILTER_ORDER/2];
	int enabled;
} filter_state;

void bwlpf_init(filter_state * filter, uint16_t sampling_frequency, uint8_t half_power_freq);
uint16_t bwlpf(uint16_t next_sample, filter_state * filter );
void bwlpf_set_freq(filter_state * filter, uint16_t sampling_frequency);



#endif /* BWLPF_H_ */