/* fft.h 

   Header file for the FFT routines 

   Created: JDB, 20000810

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

   Copyright (C)2001 Jan-Derk Bakker, jdb@lartmaker.nl

*/

#ifndef __JD_FFT_H
#define __JD_FFT_H

#if 1
#define CONST2FIX(x) ((fixpoint)((x) * (1 << EXP)))
#define FIX2CONST(x) ((int) ((x) >> EXP))
#define FIX2DOUBLE(x) (((double) (x)) / (1 << EXP))
#define DOUBLE2FIX(x) ((fixpoint)(((double)(x)) * (1 << EXP)))
#define FMUL(a,b) ((a)*(b)>>(EXP))
#define EXP 12
#define FIXP_EPSILON (1.0 / (1UL << EXP))

typedef long fixpoint;
typedef long fixpoint_accu;

#else

#define CONST2FIX(x) ((fixpoint)(x))
#define FIX2CONST(x) ((int) (x))
#define FIX2DOUBLE(x) (x)
#define DOUBLE2FIX(x) (x)
#define FMUL(a,b) ((a)*(b))
#define EXP 0
#define FIXP_EPSILON 0

typedef double fixpoint;
typedef double fixpoint_accu;

#endif

struct complex
{
		fixpoint r,i;
};

/* testfft.c */
void ReferenceDFT(struct complex *in, struct complex *out, int numPoints);
void FillVectorRandom(struct complex *in, int numPoints);
double CompareVectors(struct complex *in1, struct complex *in2, int numPoints);
double VectorMeanEnergy(struct complex *in, int numPoints);

/* radix4fft.c */
void InitFFTTables(void);
void DoFFT(struct complex *io, int numPoints);

/* radix4fft_slow.c */

#endif
