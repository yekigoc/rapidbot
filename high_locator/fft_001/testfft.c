/* testfft.c
   
   Routines to test a FFT implementation
   
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "fft.h"


static double ComplexMagnitudeSq(struct complex *in);
static double ComplexDifferenceMagnitudeSq(struct complex *in1, struct complex *in2);
static double VectorDifferenceEnergy(struct complex *in1, struct complex *in2, int numPoints);


void ReferenceDFT(struct complex *in, struct complex *out, int numPoints) {
/* Textbook-standard DFT implementation. Used to generate a reference to
   test FFTs against. Implementation is *very* non-optimal. All internal 
   calculations are in double. */
  
  double realRes, imagRes;
  int f, t;
  
  /* Loop over all output frequencies */
  for(f = 0; f < numPoints; f++) {
    realRes = imagRes = 0.0;
    
    /* Execute H(f) = sum(h(t) * exp(2*pi*i*t*f/N))
       = sum(h(t) * (cos(2*pi*t*f/N) + i * sin(2*pi*t*f/N))) */
    for(t = 0; t < numPoints; t++) {
      
      realRes += FIX2DOUBLE(in[t].r) * cos(2*M_PI*t*f/numPoints);
      imagRes += FIX2DOUBLE(in[t].i) * cos(2*M_PI*t*f/numPoints);
      
      realRes -= FIX2DOUBLE(in[t].i) * sin(2*M_PI*t*f/numPoints);
      imagRes += FIX2DOUBLE(in[t].r) * sin(2*M_PI*t*f/numPoints);
      
    }
    
    out[f].r = CONST2FIX(realRes / sqrt(numPoints));
    out[f].i = CONST2FIX(imagRes / sqrt(numPoints));
    
  }
  
}  /* ReferenceDFT */


void FillVectorRandom(struct complex *in, int numPoints) {
  /* Fill a vector of length numPoints with uniform distribited random complex
     numbers where the real and imaginary parts are inside [-1,1]. */
  
  int i;
  double factor = sqrt(3.0 / 2.0);
  
  for(i = 0; i < numPoints; i++) {
    in[i].r = CONST2FIX(rand() * 2.0 * factor / RAND_MAX - factor);
    in[i].i = CONST2FIX(rand() * 2.0 * factor / RAND_MAX - factor);
  }
  
} /* FillVectorRandom */


#define EPSILON 1e-9

double CompareVectors(struct complex *in1, struct complex *in2, int numPoints) {
  /* Determine the signal to noise ratio of a vector, by assuming in1 is the signal 
     and in2 is signal + noise. Returned value is the ratio between signal and noise power. */
  
  double energy1, difEnergy;
  
  energy1 = VectorMeanEnergy(in1, numPoints);
  difEnergy = VectorDifferenceEnergy(in1, in2, numPoints);
  if(difEnergy < EPSILON)
    difEnergy = EPSILON;
  
  return energy1 / difEnergy;
  
}  /* CompareVectors */


/* Local helper functions */

static double ComplexMagnitudeSq(struct complex *in) {
  /* Determine the squared magnitude of a complex number. Returns a nonnegative double */
  
  return FIX2DOUBLE(in->r) * FIX2DOUBLE(in->r) 
    + FIX2DOUBLE(in->i) * FIX2DOUBLE(in->i);
  
}  /* ComplexMagnitudeSq */


double VectorMeanEnergy(struct complex *in, int numPoints) {
  /* Calculate the mean per-element energy of vector in. */
  
  double result = 0.0;
  int i;
  
  for(i = 0; i < numPoints; i++)
    result += ComplexMagnitudeSq(&in[i]) / numPoints;
  
  return result;
  
} /* VectorMeanSq */


static double ComplexDifferenceMagnitudeSq(struct complex *in1, struct complex *in2) {
/* Determine the squared magnitude of the difference between two complex numbers. 
   Returns a nonnegative double */
  
  struct complex temp;
  
  temp.r = in1->r - in2->r;
  temp.i = in1->i - in2->i;
  
  return ComplexMagnitudeSq(&temp);
  
}  /* ComplexDifferenceMagnitudeSq */


static double VectorDifferenceEnergy(struct complex *in1, struct complex *in2, int numPoints) {
  /* Calculate the mean per-element energy of the difference of in1 and in2. */
  
  double result = 0.0;
  int i;
  
  for(i = 0; i < numPoints; i++)
    result += ComplexDifferenceMagnitudeSq(&in1[i], &in2[i]) / numPoints;
  
  return result;
  
} /* VectorDifferenceEnergy */

