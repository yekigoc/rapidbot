//  *****************************************************************************
//   						blinker.c
// 
//     Endless loop blinks a code for crash analysis
//     
//	   Inputs:	Code  -  blink code to display
//						 1 = undefined instruction (one blinks ........ long pause)
//						 2 = prefetch abort        (two blinks ........ long pause)
//						 3 = data abort            (three blinks ...... long pause)
//
//  Author:  James P Lynch  July 12, 2006
//  ***************************************************************************** 

#include "AT91SAM7S256.h"
#include "Board.h"

unsigned long	blinkcount;									// global variable


void  blinker(unsigned char    code) {
	volatile AT91PS_PIO		pPIO = AT91C_BASE_PIOA;			// pointer to PIO register structure
	volatile unsigned int	j,k;							// loop counters
		
	// endless loop	
	while (1)  {	
		for  (j = code; j != 0; j--) {						// count out the proper number of blinks
			pPIO->PIO_CODR = LED1;							// turn LED1 (DS1) on	
			for (k = 600000; k != 0; k-- );					// wait 250 msec
			pPIO->PIO_SODR = LED1;							// turn LED1 (DS1) off
			for (k = 600000; k != 0; k-- );					// wait 250 msec
		}
		for (k = 5000000; (code != 0) && (k != 0); k-- );	// wait 2 seconds
		blinkcount++;
	}	
}
