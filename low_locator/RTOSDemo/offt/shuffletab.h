/* shuffletab.h

   Tables to reorder the output of the FFT routines. Used
   by radix4fft.c

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

#ifndef __SHUFFLETAB__
#define __SHUFFLETAB__

/* Tables used by reorder_generic */

static unsigned char fft20shuf[] = {
   1,  4, 16,  1,  2,  8, 11,  2,  
   3, 12,  6,  3,  5, 19, 13, 10, 
  18,  9, 15, 17,  5, 0 };

static unsigned char fft80shuf[] = {  
   1, 16,  4, 64, 61, 13, 40, 78, 
  33, 39, 51,  6, 12, 24, 63, 45, 
  74, 53, 38, 35, 67, 25, 75, 65, 
  73, 37, 19, 52, 22, 31,  7, 28, 
  43, 46, 10, 72, 21, 15, 68, 41, 
  14, 56,  2, 32, 23, 47, 26, 11, 
   8, 44, 62, 29, 59, 50, 70, 69,
  57, 18, 36,  3, 48, 42, 30, 71,
   5, 76,  1,  9, 60, 77, 17, 20, 
  79, 49, 58, 34, 55, 66,  9,  0 };

static unsigned char fft64shuf[] = {
   1, 48, 61, 33, 46, 18, 31,  3, 
  16, 63,  1,  2, 32, 62, 17, 47,  
   2,  4, 60, 49, 45, 34, 30, 19, 
  15,  4,  5, 44, 50, 29, 35, 14, 
  20, 59,  5,  6, 28, 51, 13, 36, 
  58, 21, 43,  6,  7, 12, 52, 57, 
  37, 42, 22, 27,  7,  8, 56, 53, 
  41, 38, 26, 23, 11,  8,  9, 40, 
  54, 25, 39, 10, 24, 55,  9, 0 } ;


#endif
