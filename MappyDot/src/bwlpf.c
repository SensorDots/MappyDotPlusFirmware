/*
 *                            COPYRIGHT
 *
 *  Copyright (C) 2014 Exstrom Laboratories LLC
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  A copy of the GNU General Public License is available on the internet at:
 *  http://www.gnu.org/copyleft/gpl.html
 *
 *  or you can write to:
 *
 *  The Free Software Foundation, Inc.
 *  675 Mass Ave
 *  Cambridge, MA 02139, USA
 *
 *  Exstrom Laboratories LLC contact:
 *  stefan(AT)exstrom.com
 *
 *  Exstrom Laboratories LLC
 *  Longmont, CO 80503, USA
 *
 *
 *  Modified to be more modular rather than just taking arguements and data from stdin: Blair Wyatt.
 */

//http://www.exstrom.com/journal/sigproc/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bwlpf.h"

void bwlpf_init(filter_state * filter, uint16_t sampling_frequency, uint8_t half_power_freq)
{
    //n = filter order 2,4,6,...
    //s = sampling frequency
    //f = half power frequency

    int i = 0;
    filter->n = FILTER_ORDER/2;
    filter->s = sampling_frequency;
    filter->f = half_power_freq;
    filter->a = tan(M_PI*filter->f/filter->s);
    filter->a2 = filter->a*filter->a;
    filter->r = 0;
	/*if (full_init) {
		filter->A = (double *)malloc(filter->n*sizeof(double));
		filter->d1 = (double *)malloc(filter->n*sizeof(double));
		filter->d2 = (double *)malloc(filter->n*sizeof(double));
		filter->w0 = (double *)calloc(filter->n, sizeof(double));
		filter->w1 = (double *)calloc(filter->n, sizeof(double));
		filter->w2 = (double *)calloc(filter->n, sizeof(double));
	}*/


    for(i=0; i<filter->n; ++i)
    {
        filter->r = sin(M_PI*(2.0*i+1.0)/(4.0*filter->n));
        filter->s = filter->a2 + 2.0*filter->a*filter->r + 1.0;
        filter->A[i] = filter->a2/filter->s;
        filter->d1[i] = 2.0*(1-filter->a2)/filter->s;
        filter->d2[i] = -(filter->a2 - 2.0*filter->a*filter->r + 1.0)/filter->s;
        //Initialise as 0 to stop buffer overflow
        filter->w0[i] = 0;
        filter->w1[i] = 0;
        filter->w2[i] = 0;
    }

	if (sampling_frequency < 12) //If less than 12Hz, disable
		filter->enabled = 0;
	else
		filter->enabled = 1;
	
}

uint16_t bwlpf(uint16_t next_sample, filter_state * filter )
{
    int i = 0;
    double x = next_sample;
	if (filter->enabled) {
		for(i=0; i<filter->n; ++i)
		{
			filter->w0[i] = filter->d1[i]*filter->w1[i] + filter->d2[i]*filter->w2[i] + x;
			x = filter->A[i]*(filter->w0[i] + 2.0*filter->w1[i] + filter->w2[i]);
			filter->w2[i] = filter->w1[i];
			filter->w1[i] = filter->w0[i];
		}
	}
    return (uint16_t)x;
}
