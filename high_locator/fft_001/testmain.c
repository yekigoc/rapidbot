/* testmain.c
   
   Routines to test a FFT implementation
   
   Created: JDB, 20001212

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
#include <math.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>


#include "fft.h"


#define FFT_SIZE 64
#define FREQ 0
#define ITERS 100000

static void TransTable(void);
static void SinTabTest(void);

int main(void) {
  
  struct complex in[FFT_SIZE], out_ref[FFT_SIZE], out_test[FFT_SIZE];
  double snr;
  int i, j;
  double elapsed;
  struct timeval start, end;

#if 0
  SinTabTest();
#endif
  printf("Testing a %d-point FFT.\n", FFT_SIZE);
  InitFFTTables();
#if 1
  TransTable();
#endif

#if 1
  FillVectorRandom(in, FFT_SIZE);
#else
  for(i = 0; i < FFT_SIZE; i++) {
    in[i].r = DOUBLE2FIX(cos(2 * M_PI * i * FREQ / FFT_SIZE));
    in[i].i = DOUBLE2FIX(-sin(2 * M_PI * i * FREQ / FFT_SIZE));
  }
#endif
  
  ReferenceDFT(in, out_ref, FFT_SIZE);

  for(i = 0; i < FFT_SIZE; i++) {
    out_test[i].r = in[i].r;
    out_test[i].i = in[i].i;
  }

  DoFFT(out_test, FFT_SIZE);

#if 0
  for(i = 0; i < FFT_SIZE; i++) {
    printf("%3d (%f,%f) \t: (%f,%f)\t- (%f,%f)\n", i,
	   FIX2DOUBLE(in[i].r), FIX2DOUBLE(in[i].i), 
	   FIX2DOUBLE(out_test[i].r), FIX2DOUBLE(out_test[i].i), 
	   FIX2DOUBLE(out_ref[i].r), FIX2DOUBLE(out_ref[i].i));
  }
#endif  

  snr = CompareVectors(out_ref, out_test, FFT_SIZE);
  printf("SNR: %f (%f dB)\n", snr, 10.0 * log10(snr));
  printf("Mean energy in: %f out_test: %f out_ref: %f\n",
	 VectorMeanEnergy(in, FFT_SIZE), 
	 VectorMeanEnergy(out_test, FFT_SIZE), 
	 VectorMeanEnergy(out_ref, FFT_SIZE));

  printf("Timing FFT speed...");
  fflush(stdout);
  for(i = 0; i < FFT_SIZE; i++)
    out_test[i].r = out_test[i].i = 0;
  gettimeofday(&start, (struct timezone *) 0);
  for(i = 0; i < ITERS; i++)
    DoFFT(out_test, FFT_SIZE);
  gettimeofday(&end, (struct timezone *) 0);
  elapsed = 1000000 * (end.tv_sec - start.tv_sec) 
    + (end.tv_usec - start.tv_usec);

  printf(" %.2f us per %d-point FFT, or %.2f Mbps with QPSK,\n"
	 "or %.2f insns per point on a 220MHz SA-1100.\n", 
	 (double)elapsed / ITERS, FFT_SIZE, 
	 2.0 * FFT_SIZE * ITERS / elapsed,
	 (double) elapsed * 3.6864 * 60 / ITERS / FFT_SIZE);

  return 0;
}


static void TransTable(void) {

  struct complex in[FFT_SIZE];
  int i, j, maxind, numInvariants, startind, nextind;
  int transtab[FFT_SIZE], invtab[FFT_SIZE], iffttab[FFT_SIZE], inviffttab[FFT_SIZE];
  
  /* Determine the translation table */
  for(i = 0; i < FFT_SIZE; i++) {
    for(j = 0; j < FFT_SIZE; j++) {
      in[j].r = DOUBLE2FIX(cos(2 * M_PI * i * j / FFT_SIZE));
      in[j].i = DOUBLE2FIX(-sin(2 * M_PI * i * j / FFT_SIZE));
    }
   DoFFT(in, FFT_SIZE);
    maxind = 0;
    for(j = 1; j < FFT_SIZE; j++)
      if(in[j].r > in[maxind].r) {
	maxind = j;
      }
    transtab[i] = maxind;
    if(i == 0)
      iffttab[i] = maxind;
    else
      iffttab[FFT_SIZE - i] = maxind;
  }
  numInvariants = 0;
  for(i = 0; i < FFT_SIZE; i++) {
    for(j = i + 1; j < FFT_SIZE; j++) {
      if(transtab[i] == transtab[j]) {
	printf("*** WARNING: table entries %d and %d collide !\n", i, j);
      }
    }
    invtab[transtab[i]] = i;
    inviffttab[iffttab[i]] = i;
    if(transtab[i] == i)
      numInvariants++;
  }
  if(numInvariants == FFT_SIZE)
    printf("1:1 mapping; no reorder required.\n");
  else {
    printf("In-place FFT table:\n{\n");
    for(i = 1; i < FFT_SIZE; i++) {
      if(invtab[i] < 0)
	continue;
      startind = i;
      printf("%2d, ", startind);
      do {
	nextind = invtab[i];
	assert(nextind >= 0);
	invtab[i] = -invtab[i];
	i = nextind;
	printf("%2d, ", i);
      } while(i != startind);
    }
    printf("0 }\n");
    printf("Out-of-place FFT table (dst[i] = src[tab[i]]):\n{\n%2d", transtab[0]);
    for(i = 1; i < FFT_SIZE; i++)
      printf(", %2d", transtab[i]);
    printf(" }\n");
    
    printf("In-place IFFT table:\n{\n");
    for(i = 1; i < FFT_SIZE; i++) {
      if(inviffttab[i] < 0)
	continue;
      startind = i;
      printf("%2d, ", startind);
      do {
	nextind = inviffttab[i];
	assert(nextind >= 0);
	inviffttab[i] = -inviffttab[i];
	i = nextind;
	printf("%2d, ", i);
      } while(i != startind);
    }
    printf("0 }\n");
    printf("Out-of-place IFFT table (dst[i] = src[tab[i]]):\n{\n%2d", iffttab[0]);
    for(i = 1; i < FFT_SIZE; i++)
      printf(", %2d", iffttab[i]);
    printf(" }\n");    
  }
}


#define SINVECLEN 256

static void SinTabTest(void) {

  struct complex orig[SINVECLEN], tabbed[SINVECLEN];
  int i, tabSize;

  for(i = 0; i < SINVECLEN; i++) {
    orig[i].r = DOUBLE2FIX(sin(2 * M_PI * i / SINVECLEN));
    orig[i].i = DOUBLE2FIX(cos(2 * M_PI * i / SINVECLEN));
  }
  printf("Energy in original sine: %f.\n", VectorMeanEnergy(orig, SINVECLEN));

  for(tabSize = 256; tabSize < 5000; tabSize *= 2) {
    for(i = 0; i < SINVECLEN; i++) {
      tabbed[i].r = DOUBLE2FIX(sin(2 * M_PI * (i + 128.0 / tabSize) / SINVECLEN));
      tabbed[i].i = DOUBLE2FIX(cos(2 * M_PI * (i + 128.0 / tabSize) / SINVECLEN));
    }
    printf("Max full cycle distortion for a table of %d : %f (%f).\n",
	   tabSize, 10.0 * log10(CompareVectors(orig, tabbed, SINVECLEN)),
	   VectorMeanEnergy(tabbed, SINVECLEN));
  }
} /* SinTabTest */


