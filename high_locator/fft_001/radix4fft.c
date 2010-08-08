/* radix4fft.c

   Fast routines to do mixed radix-4/5 DIF FFTs. Supported sizes
   include 20, 64 and 80 complex points.

   Optimized for ARM. If possible compile with -fomit-frame-pointer , 
   as this gives the compiler another register to work with.

   Created: 20001212, JDB.

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
#include "bflymacro.h"
#include "shuffletab.h"


/* Static functions */
static void twiddle(struct complex *tw, int N, double f_t);
static void reorder_generic(fixpoint *io, unsigned char *shuffletab);
static void fft64(fixpoint *io);
static void fft64_rev(fixpoint *io);
static void fft5_inittab(fixpoint *B, double scale);
static void fft4_inittab(fixpoint *twtab, int numPoints);
static void fft20(fixpoint *io, fixpoint *twtab, int numIters);
static void fft80(fixpoint *io, fixpoint *twtab);
static void dofft64(fixpoint *io);
static void dofft20(fixpoint *io);
static void dofft80(fixpoint *io);


/* Static twiddle/multiplication tables */
static fixpoint fft5_B_20[6];
static fixpoint fft5_B_80[6];

#define TWTAB_SIZE(x) (((x) / 4) * 3 * 2)

static fixpoint tw_fft80[TWTAB_SIZE(80)];
static fixpoint tw_fft20[TWTAB_SIZE(20)];
static fixpoint tw_fft4[TWTAB_SIZE(4)];
static fixpoint tw_fft16[TWTAB_SIZE(16)];
static fixpoint tw_fft64[TWTAB_SIZE(64)];


void InitFFTTables(void) {

  fft5_inittab(fft5_B_20, 1);
  fft5_inittab(fft5_B_80, 0.5);

  fft4_inittab(tw_fft80, 80);
  fft4_inittab(tw_fft20, 20);
  fft4_inittab(tw_fft4, 4);
  fft4_inittab(tw_fft16, 16);
  fft4_inittab(tw_fft64, 64);

} /* InitFFTTables */


void DoFFT(struct complex *io, int numPoints) {

  switch(numPoints) {
  case 20:
    dofft20((fixpoint *) io);
    break;
  case 64:
    dofft64((fixpoint *) io);
    break;
  case 80:
    dofft80((fixpoint *) io);
    break;
  default:
    fprintf(stderr, "FFT size %d not implemented.\n", numPoints);
    exit(2);
  }
} /* DoFFT */


static void twiddle(struct complex *tw, int N, double f_t) {

#if (EXP >= 31)
  double val;

  val = cos(2 * M_PI * f_t / N);
  if(val > (1.0 - FIXP_EPSILON))
    val = 1.0 - FIXP_EPSILON;
  if(val < -1.0)
    val = -1.0;
  tw->r = DOUBLE2FIX(val);
  val = -sin(2 * M_PI * f_t / N);
  if(val > (1.0 - FIXP_EPSILON))
    val = 1.0 - FIXP_EPSILON;
  if(val < -1.0)
    val = -1.0;
  tw->i = DOUBLE2FIX(val);
#else
  tw->r = DOUBLE2FIX(cos(2 * M_PI * f_t / N));
  tw->i = DOUBLE2FIX(-sin(2 * M_PI * f_t / N));
#endif
  
} /* twiddle */


static void reorder_generic(fixpoint *io, unsigned char *shuffletab) {

#if 1
  fixpoint ping_r, ping_i, pong_r, pong_i;
  fixpoint *base_r = io, *base_i = io + 1;
  int curfix = *shuffletab;

  ping_r = base_r[2 * curfix];
  ping_i = base_i[2 * curfix];
  do {
    pong_r = base_r[2 * curfix];
    pong_i = base_i[2 * curfix];
    base_r[2 * curfix] = ping_r;
    base_i[2 * curfix] = ping_i;
    curfix = *shuffletab++;
    ping_r = base_r[2 * curfix];
    ping_i = base_i[2 * curfix];
    base_r[2 * curfix] = pong_r;
    base_i[2 * curfix] = pong_i;
    curfix = *shuffletab++;
  } while(curfix != 0);
#endif

} /* reorder_generic */ 


static void fft4_gen_size(fixpoint *io, int numPoints) {
/* Generic radix-4 FFT optimized for size. Good when you're low on cache; 
   not very fast though */

  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7;
  register fixpoint_accu accu;
  register fixpoint *twtab;
  fixpoint *twtabs[3] = { tw_fft4, tw_fft16, tw_fft64 };
  int numIters = 1, iter, numBlocks, block, numBflys, bfly;
  int stride, offset, base;

  while((1 << (2 * numIters)) < numPoints)
    numIters++;

  for(iter = 0; iter < numIters; iter++) {
    numBlocks = 1 << (2 * iter);
    numBflys = (numPoints / 4) / numBlocks;
    stride = numBflys;
    for(block = 0; block < numBlocks; block++) {
      offset = 4 * block * numBflys;
      twtab = twtabs[numIters - 1 - iter];
      for(bfly = 0; bfly < numBflys; bfly++) {
	    base = offset + bfly;
#if 0
	    printf("Butterfly lvl %d (%2d, %2d, %2d, %2d)\tmul ", iter, 
	           base, base + stride, base + 2*stride, base + 3*stride);
	    printf("0 ");
	    printf("%d %d %d ", bfly, bfly * 2, bfly * 3);
	    printf("/ %d JD\n", 4 * numBflys);
#endif
        DIF_BFLY4(io, io, base, stride, 1, twtab);
      }
    }
  }

} /* fft4_gen_size */


static void fft64(fixpoint *io) {
  fixpoint *io_end, *io_orig = io;
  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7;
  register fixpoint_accu accu;
  register fixpoint *twtab;

  /* Level 1 */
  twtab = tw_fft64;
  io -= 2;
  io_end = io + 32;
  do {
    io += 2;
    DIF_BFLY4(io, io, 0, 16, 0, twtab);
  } while (io < io_end);

  /* Level 2 */
  twtab = tw_fft16;
  io = io_orig - 2;
  io_end = io + 128;
  do {
    io += 2;
    DIF_BFLY4(io, io, 0, 4, 0, twtab);
    if((io_end - io) % 8 == 0) {	
      twtab -= TWTAB_SIZE(16);
      io += 24;
	};
  } while(io < io_end);

  /* Level 3 */
  io = io_orig - 8;
  io_end = io + 128;
  do {
    io += 8;
    DIF_BFLY4NOMUL(io, io, 0, 1, 3);
  } while (io < io_end);

} /* fft64 */


static void fft64_rev(fixpoint *io) {
  int base;
  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7;
  register fixpoint_accu accu;
  register fixpoint *twtab;

  /* Level 1 */
  twtab = tw_fft16;
  for(base = 0; base < 128; base += 8) {
    DIF_BFLY4(io, io, base, 1, 0, twtab);
  }

  /* Level 2 */
  twtab = tw_fft64;
  for(base = 0; base < 128; base += 2) {
    DIF_BFLY4(io, io, base, 4, 0, twtab);
    if((base & 6) == 6) {
      base += 24;
    }
  }

  /* Level 3 */
  for(base = 0; base < 32; base += 2) {
    DIF_BFLY4NOMUL(io, io, base, 16, 3);
  }

} /* fft64_rev */


static void dofft64(fixpoint *io) {

  fft64(io);
  reorder_generic((fixpoint *)io, fft64shuf);

} /* dofft64 */


#define THETA (2.0 * M_PI / 5.0)
#define F5SCALE (2.0 / sqrt(5))

static void fft5_inittab(fixpoint *B, double scale) {

  B[0] = DOUBLE2FIX(1.0 * F5SCALE * scale);
  B[1] = DOUBLE2FIX((0.5 * (cos(THETA) + cos(2 * THETA)) - 1) * F5SCALE * scale);
  B[2] = DOUBLE2FIX((0.5 * (cos(THETA) - cos(2 * THETA))) * F5SCALE * scale);
  B[3] = DOUBLE2FIX(sin(THETA) * F5SCALE * scale);
  B[4] = DOUBLE2FIX((sin(THETA) + sin(2 * THETA)) * F5SCALE * scale);
  B[5] = DOUBLE2FIX((sin(2 * THETA) - sin(THETA)) * F5SCALE * scale);

} /* fft5_inittab */


static void fft5(fixpoint *io, fixpoint *B, int numIters) {
  /* Do a 5-point FFT on the input data */

  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10;
 
  do {
    DIF_BFLY5NOMUL(io, 2);
  } while(--numIters > 0);

} /* fft_5 */


static void fft4_inittab(fixpoint *twtab, int numPoints) {

  int bfly;
  struct complex tw;

  for(bfly = 0; bfly < numPoints / 4; bfly++) {
    twiddle(&tw, numPoints, bfly);
    *twtab++ = tw.r;
    *twtab++ = tw.i;
    twiddle(&tw, numPoints, bfly * 2);
    *twtab++ = tw.r;
    *twtab++ = tw.i;
    twiddle(&tw, numPoints, bfly * 3);
    *twtab++ = tw.r;
    *twtab++ = tw.i;
  }
} /* fft4_inittab */


static void fft20(fixpoint *io, fixpoint *twtab, int numIters) {

  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7;
  register fixpoint_accu accu;
  int i;

  do {
    for(i = 0; i < 10; i += 2)
      DIF_BFLY4(io, io, i, 5, 0, twtab);
    twtab -= TWTAB_SIZE(20);
    io += 40;
  } while(--numIters > 0);

} /* fft20 */


static void dofft20(fixpoint *io) {

  fft20(io, tw_fft20, 1);
  fft5(io, fft5_B_20, 4);
  reorder_generic(io, fft20shuf);

} /* dofft20 */


static void fft80(fixpoint *io, fixpoint *twtab) {

  register fixpoint r0, r1, r2, r3, r4, r5, r6, r7;
  register fixpoint_accu accu;
  int i;

  for(i = 0; i < 40; i += 2)
    DIF_BFLY4(io, io, i, 20, 0, twtab);

} /* fft80 */


static void dofft80(fixpoint *io) {

  fft80(io, tw_fft80);
  fft20(io, tw_fft20, 4);
  fft5(io, fft5_B_80, 16);

  reorder_generic(io, fft80shuf);

} /* dofft80 */

