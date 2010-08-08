/* bflymacro.h

   Contains macros for radix-4/5 butterflies,
   and some useful derivates. Most routines are based
   on the algorithms in "Fast algorithms for digital 
   signal processing" by Richard E. Blahut, published
   by Addison-Wesley Publishing Company.

   The macros are optimized for ARM CPUs, but should
   work well on any register-rich architecture.

   Created: 20001227, JDB.

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

#ifndef __BFLYMACRO__
#define __BFLYMACRO__

/* All DI*_BFLY4* macros require register fixpoint r0...r7 and accu to
   be declared in the including function. */

#define BFLY4CORE do {   	\
	r0 = r0 + r4;				\
	r1 = r1 + r5;				\
	r4 = r0 - (r4 << 1);			\
	r5 = r1 - (r5 << 1);			\
	r2 = r2 + r6;				\
	r3 = r3 + r7;				\
	r6 = r2 - (r6 << 1);			\
	r7 = r3 - (r7 << 1);			\
	r0 = r0 + r2;				\
	r1 = r1 + r3;				\
	r2 = r0 - (r2 << 1);			\
	r3 = r1 - (r3 << 1);			\
	/* 12 insns */				\
						\
	r4 = r4 + r7;				\
	r5 = r5 + r6;				\
	r6 = r5 - (r6 << 1);			\
	r7 = r4 - (r7 << 1);			\
	/* 4 insns */				\
     } while(0)


#define DIF_LOAD4(INBUF,BASE,STRIDE) do {   	        \
						\
	r0 = INBUF[BASE];	       		\
	r1 = INBUF[BASE + 1];			\
	r2 = INBUF[BASE + 2*STRIDE];		\
	r3 = INBUF[BASE + 2*STRIDE + 1];		\
	r4 = INBUF[BASE + 4*STRIDE];		\
	r5 = INBUF[BASE + 4*STRIDE + 1];		\
	r6 = INBUF[BASE + 6*STRIDE];		\
	r7 = INBUF[BASE + 6*STRIDE + 1];		\
	/* 8 insns */				\
     } while(0)

#define DIF_BFLY4(INBUF,OUTBUF,BASE,STRIDE,SHIFT,TWIDDLE) do {   	        \
						\
	DIF_LOAD4(INBUF,BASE,STRIDE);   		\
	BFLY4CORE;   		\
						\
	r0 >>= SHIFT;				\
	r1 >>= SHIFT;				\
	OUTBUF[BASE] = r0;		       	\
	OUTBUF[BASE + 1] = r1;			\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r4 * r1;				\
	accu = r6 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 2*STRIDE + 1] = accu;		\
	r1 = -r1;				\
	accu = r6 * r1;				\
	accu = r4 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 2*STRIDE] = accu;		\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r2 * r1;				\
	accu = r3 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 4*STRIDE + 1] = accu;		\
	r1 = -r1;				\
	accu = r3 * r1;				\
	accu = r2 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 4*STRIDE] = accu;		\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r7 * r1;				\
	accu = r5 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 6*STRIDE + 1] = accu;		\
	r1 = -r1;				\
	accu = r5 * r1;				\
	accu = r7 * r0 + accu;			\
	accu >>= (SHIFT + EXP);				\
	OUTBUF[BASE + 6*STRIDE] = accu;		\
	/* 37 insns */				\
     } while(0)

#define DIF_BFLY4NOMUL(INBUF,OUTBUF,BASE,STRIDE,SHIFT) do { \
						\
	DIF_LOAD4(INBUF,BASE,STRIDE);   		\
	BFLY4CORE;   		\
						\
	r0 >>= SHIFT;				\
	r1 >>= SHIFT;				\
	OUTBUF[BASE] = r0;		       	\
	OUTBUF[BASE + 1] = r1;			\
	r4 >>= SHIFT;				\
	r6 >>= SHIFT;				\
	OUTBUF[BASE + 2*STRIDE] = r4;		\
	OUTBUF[BASE + 2*STRIDE + 1] = r6;		\
	r2 >>= SHIFT;				\
	r3 >>= SHIFT;				\
	OUTBUF[BASE + 4*STRIDE] = r2;		\
	OUTBUF[BASE + 4*STRIDE + 1] = r3;		\
	r7 >>= SHIFT;				\
	r5 >>= SHIFT;				\
	OUTBUF[BASE + 6*STRIDE] = r7;		\
	OUTBUF[BASE + 6*STRIDE + 1] = r5;		\
	/* 16 insns */				\
     } while(0)


/* Radix-4 Decimation-in-time macros */

#define DIT_STORE4(OUTBUF,BASE,STRIDE) do {   	        \
						\
	OUTBUF[BASE] = r0;		       	\
	OUTBUF[BASE + 1] = r1;			\
	OUTBUF[BASE + 2*STRIDE] = r4;		\
	OUTBUF[BASE + 2*STRIDE + 1] = r6;		\
	OUTBUF[BASE + 4*STRIDE] = r2;		\
	OUTBUF[BASE + 4*STRIDE + 1] = r3;		\
	OUTBUF[BASE + 6*STRIDE] = r7;		\
	OUTBUF[BASE + 6*STRIDE + 1] = r5;		\
	/* 8 insns */				\
     } while(0)

#define DIT_BFLY4(INBUF,OUTBUF,BASE,STRIDE,SHIFT,TWIDDLE) do {   	        \
						\
	r2 = INBUF[BASE + 2*STRIDE];		\
	r3 = INBUF[BASE + 2*STRIDE + 1];		\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r2 * r1;				\
	r1 = -r1;				\
	r1 = r3 * r1;				\
	r2 = r2 * r0 + r1;			\
	r3 = r3 * r0 + accu;			\
	r2 >>= (SHIFT + EXP);		\
	r3 >>= (SHIFT + EXP);		\
	/* 11 insns */				\
						\
	r4 = INBUF[BASE + 2*STRIDE];		\
	r5 = INBUF[BASE + 2*STRIDE + 1];		\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r4 * r1;				\
	r1 = -r1;				\
	r1 = r5 * r1;				\
	r4 = r4 * r0 + r1;			\
	r5 = r5 * r0 + accu;			\
	r4 >>= (SHIFT + EXP);		\
	r5 >>= (SHIFT + EXP);		\
	/* 11 insns */				\
						\
	r6 = INBUF[BASE + 2*STRIDE];		\
	r7 = INBUF[BASE + 2*STRIDE + 1];		\
	r0 = *TWIDDLE++;				\
	r1 = *TWIDDLE++;				\
	accu = r6 * r1;				\
	r1 = -r1;				\
	r1 = r7 * r1;				\
	r6 = r6 * r0 + r1;			\
	r7 = r7 * r0 + accu;			\
	r6 >>= (SHIFT + EXP);		\
	r7 >>= (SHIFT + EXP);		\
	/* 11 insns */				\
						\
	r0 = INBUF[BASE];	       		\
	r1 = INBUF[BASE + 1];			\
	/* 2 insns */				\
						\
	BFLY4CORE;   		\
	DIT_STORE4(OUTBUF,BASE,STRIDE);		       	\
						\
     } while(0)

#define DIT_BFLY4NOMUL(INBUF,OUTBUF,BASE,STRIDE,SHIFT) do { \
						\
	r0 = INBUF[BASE];	       		\
	r1 = INBUF[BASE + 1];			\
	r2 = INBUF[BASE + 2*STRIDE];		\
	r3 = INBUF[BASE + 2*STRIDE + 1];		\
	r4 = INBUF[BASE + 4*STRIDE];		\
	r5 = INBUF[BASE + 4*STRIDE + 1];		\
	r6 = INBUF[BASE + 6*STRIDE];		\
	r7 = INBUF[BASE + 6*STRIDE + 1];		\
	r0 >>= SHIFT;				\
	r1 >>= SHIFT;				\
	r2 >>= SHIFT;				\
	r3 >>= SHIFT;				\
	r4 >>= SHIFT;				\
	r5 >>= SHIFT;				\
	r6 >>= SHIFT;				\
	r7 >>= SHIFT;				\
	/* 16 insns */				\
						\
	BFLY4CORE;   		\
	DIT_STORE4(OUTBUF,BASE,STRIDE);		       	\
						\
     } while(0)


/* DIF_BFLY5NOMUL requires register fixpoint r0...r10 to be declared.
   Note that DIF_BFLY5NOMUL auto-increments IOBUF; as BFLY5NOMUL is 
   normally only used in the final stage of a Decimation-in-frequency
   FFT this is not a problem. */

#define DIF_BFLY5NOMUL(IOBUF,SHIFT) do {		\
	r0 = IOBUF[0];				\
	r1 = IOBUF[1];				\
	r2 = IOBUF[2];				\
	r3 = IOBUF[3];				\
	r4 = IOBUF[4];				\
	r5 = IOBUF[5];				\
	r6 = IOBUF[6];				\
	r7 = IOBUF[7];				\
	r8 = IOBUF[8];				\
	r9 = IOBUF[9];				\
	/* 10 insns */				\
						\
	r2 = r2 + r8;				\
	r3 = r3 + r9;				\
	r8 = r2 - (r8 << 1);			\
	r9 = r3 - (r9 << 1);			\
	r6 = r6 + r4;				\
	r7 = r7 + r5;				\
	r4 = r6 - (r4 << 1);			\
	r5 = r7 - (r5 << 1);			\
	r2 = r2 + r6;				\
	r3 = r3 + r7;				\
	r6 = r2 - (r6 << 1);			\
	r7 = r3 - (r7 << 1);			\
	r0 = r0 + r2;				\
	r1 = r1 + r3;				\
	/* 14 insns */				\
						\
	r10 = B[0];				\
	r0 = r0 * r10; 				\
	r1 = r1 * r10; 				\
	r10 = B[1];				\
	r2 = r0 + r2 * r10;    			\
	r3 = r1 + r3 * r10;    			\
	r0 >>= (EXP + SHIFT);			\
	r1 >>= (EXP + SHIFT);			\
	*IOBUF++ = r0;				\
	*IOBUF++ = r1;				\
	/* 10 insns */				\
						\
	/* Reuse of r0/1 ! */			\
	r0 = r4 + r8;				\
	r1 = r5 + r9;				\
	r10 = B[2];				\
	r6 = r6 * r10; 				\
	r7 = r7 * r10; 				\
	r10 = B[3];				\
	r0 = r0 * r10; 				\
	r1 = r1 * r10; 				\
	r10 = B[4];				\
	r4 = r4 * r10; 				\
	r5 = r5 * r10; 				\
	r10 = B[5];				\
	r8 = r0 + r8 * r10; 	       		\
	r9 = r1 + r9 * r10;	       		\
	/* 14 insns */				\
						\
	r2 = r2 + r6;				\
	r3 = r3 + r7;				\
	r6 = r2 - (r6 << 1);			\
	r7 = r3 - (r7 << 1);			\
	r4 = r0 - r4;				\
	r5 = r1 - r5;				\
	r2 = r2 + r5;				\
	r3 = r3 - r4;				\
	r5 = r2 - (r5 << 1);			\
	r4 = r3 + (r4 << 1);			\
	r6 = r6 + r9;				\
	r7 = r7 - r8;				\
	r9 = r6 - (r9 << 1);			\
	r8 = r7 + (r8 << 1);			\
	/* 14 insns */				\
						\
	r5 >>= (EXP + SHIFT);			\
	r4 >>= (EXP + SHIFT);			\
	r9 >>= (EXP + SHIFT);			\
	r8 >>= (EXP + SHIFT);			\
	r6 >>= (EXP + SHIFT);			\
	r7 >>= (EXP + SHIFT);			\
	r2 >>= (EXP + SHIFT);			\
        r3 >>= (EXP + SHIFT);			\
	*IOBUF++ = r5;				\
	*IOBUF++ = r4;				\
	*IOBUF++ = r9;				\
	*IOBUF++ = r8;				\
	*IOBUF++ = r6;				\
	*IOBUF++ = r7;				\
	*IOBUF++ = r2;				\
	*IOBUF++ = r3;				\
	/* 16 insns */				\
      } while(0)


#endif /* __BFLYMACRO__ */
