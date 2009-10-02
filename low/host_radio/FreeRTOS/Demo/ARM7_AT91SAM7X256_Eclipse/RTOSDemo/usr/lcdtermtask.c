/*
  FreeRTOS V5.4.2 - Copyright (C) 2009 Real Time Engineers Ltd.

  This file is part of the FreeRTOS distribution.

  FreeRTOS is free software; you can redistribute it and/or modify it	under 
  the terms of the GNU General Public License (version 2) as published by the 
  Free Software Foundation and modified by the FreeRTOS exception.
  **NOTE** The exception to the GPL is included to allow you to distribute a
  combined work that includes FreeRTOS without being obliged to provide the 
  source code for proprietary components outside of the FreeRTOS kernel.  
  Alternative commercial license and support terms are also available upon 
  request.  See the licensing section of http://www.FreeRTOS.org for full 
  license details.

  FreeRTOS is distributed in the hope that it will be useful,	but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along
  with FreeRTOS; if not, write to the Free Software Foundation, Inc., 59
  Temple Place, Suite 330, Boston, MA  02111-1307  USA.


  ***************************************************************************
  *                                                                         *
  * Looking for a quick start?  Then check out the FreeRTOS eBook!          *
  * See http://www.FreeRTOS.org/Documentation for details                   *
  *                                                                         *
  ***************************************************************************

  1 tab == 4 spaces!

  Please ensure to read the configuration and relevant port sections of the
  online documentation.

  http://www.FreeRTOS.org - Documentation, latest information, license and
  contact details.

  http://www.SafeRTOS.com - A version that is certified for use in safety
  critical systems.

  http://www.OpenRTOS.com - Commercial support, development, porting,
  licensing and training services.
*/


/**
 * Creates eight tasks, each of which flash an LED at a different rate.  The first 
 * LED flashes every 125ms, the second every 250ms, the third every 375ms, etc.
 *
 * The LED flash tasks provide instant visual feedback.  They show that the scheduler 
 * is still operational.
 *
 * The PC port uses the standard parallel port for outputs, the Flashlite 186 port 
 * uses IO port F.
 *
 * \page flashC flash.c
 * \ingroup DemoFiles
 * <HR>
 */

/*
  Changes from V2.0.0

  + Delay periods are now specified using variables and constants of
  portTickType rather than unsigned portLONG.

  Changes from V2.1.1

  + The stack size now uses configMINIMAL_STACK_SIZE.
  + String constants made file scope to decrease stack depth on 8051 port.
*/

#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "./liblcd/libterm.h"

/* Demo program include files. */
//#include "print.h"

#include "lcdtermtask.h"

#define lcdSTACK_SIZE		configMINIMAL_STACK_SIZE


/* The task that is created eight times - each time with a different xLEDParaemtes 
   structure passed in as the parameter. */
static void vLCDTask( void *pvParameters );

/* String to print if USE_STDIO is defined. */
const portCHAR * const pcTaskStartMsg = "LCD task started.\r\n";

/*-----------------------------------------------------------*/

xLCDParameters * vStartLCDTasks( unsigned portBASE_TYPE uxPriority )
{
  unsigned portBASE_TYPE uxLCDTask;
  xLCDParameters *pxLCDParameters;

  /* Create and complete the structure used to pass parameters to the next 
     created task. */
  pxLCDParameters = ( xLCDParameters * ) pvPortMalloc( sizeof( xLCDParameters ) );
  pxLCDParameters->ts = ( term_stat * ) pvPortMalloc( sizeof( term_stat ) );
  pxLCDParameters->lcdinited = 0;
  
  /* Spawn the task. */
  xTaskCreate( vLCDTask, "LCD", lcdSTACK_SIZE, ( void * ) pxLCDParameters, uxPriority, ( xTaskHandle * ) NULL );
  return pxLCDParameters;
}
/*-----------------------------------------------------------*/

static void vLCDTask( void *pvParameters )
{
  xLCDParameters *pxParameters;

  /* Queue a message for printing to say the task has started. */
  //  vPrintDisplayMessage( &pcTaskStartMsg );

  pxParameters = ( xLCDParameters * ) pvParameters;
  portENTER_CRITICAL();
  if (pxParameters->lcdinited == 0)
    {
      term_init(pxParameters->ts);
      InitLCD();
      LCDSettings();
      Backlight(BKLGHT_LCD_ON);
      LCDclearbg (131,0,0,131,WHITE);
      term_pushstr(pxParameters->ts, "LCD initialized");
      pxParameters->lcdinited = 1;
    }
  portEXIT_CRITICAL();
  int toggler = 0;
  char a = ' ';
  while (1)
    {
      term_pushstr(pxParameters->ts, "test");
      if (toggler == 0 )
	{
	  toggler = 1;
	  LCDPutChar(a, 120, 120, SMALL, RED, BLACK);
	}
      else
	{
	  toggler = 0;
	  LCDPutChar(a, 120, 120, SMALL, BLACK, RED);
	}
      //      portENTER_CRITICAL();
      term_print(pxParameters->ts);
      //      portEXIT_CRITICAL();
    }
      /* Delay for half the flash period then turn the LED on. */
      //      vTaskDelay( pxParameters->xFlashRate / ( portTickType ) 2 );
}
