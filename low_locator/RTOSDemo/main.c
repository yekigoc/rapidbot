/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

//#include "partest.h"
#include "USB-CDC.h"
#include "common.h"
#include "pio/pio.h"

#include "adc/adc.h"
//#include <memory.h>

#include "locator/locator_task.h"

/* Priorities for the demo application tasks. */
#define mainUSB_PRIORITY			( tskIDLE_PRIORITY +2 )
#define mainUSB_TASK_STACK			( 200 )

/* The rate at which the idle hook sends data to the USB port. */
#define mainUSB_TX_FREQUENCY		        ( 100 / portTICK_RATE_MS )

static void prvSetupHardware( void );

int main( void )
{
  prvSetupHardware();

  memset(&trspistat, 0, sizeof(trspistat));
  trspistat.pwmp.dutycycles = 250;
  trspistat.pwmp.cyclechange = 1;
  Pin a = PA1;
  trspistat.leds[0].pioled = a ;

  //  vStartProximitySensorTask( tskIDLE_PRIORITY + 1 );
  vStartLocatorTask( tskIDLE_PRIORITY + 1 );
  xTaskCreate( vUSBCDCTask, ( signed portCHAR * ) "USB", mainUSB_TASK_STACK, NULL, mainUSB_PRIORITY, NULL );

  vTaskStartScheduler();
  return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  portDISABLE_INTERRUPTS();
	
  AT91C_BASE_AIC->AIC_EOICR = 0;

/*  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;
  PWMC_ConfigureClocks(PWM_FREQUENCY * MAX_DUTY_CYCLE, 0, BOARD_MCK);
*/
  PIO_Configure(pins, PIO_LISTSIZE(pins));
  /*  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;

  PWMC_ConfigureChannel(CHANNEL_PWM_1, AT91C_PWMC_CPRE_MCKA, 0, 0);
  PWMC_SetPeriod(CHANNEL_PWM_1, MAX_DUTY_CYCLE);
  PWMC_SetDutyCycle(CHANNEL_PWM_1, MIN_DUTY_CYCLE);

  PWMC_EnableChannel(CHANNEL_PWM_1);*/

  //  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
  //adcinit();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
  static portTickType xLastTx = 0;
  portCHAR cTxByte;
  
  //  trspistat.counter = xTaskGetTickCount();

/*  if ((trspistat.pwmp.cyclechange & 1<<1) !=0)
    {
      PWMC_SetDutyCycle(CHANNEL_PWM_1, trspistat.pwmp.dutycycles);
      trspistat.pwmp.cyclechange&=~1<<1;
    }*/



  int i = 0;
  
  /*  if (trspistat.leds[0].state==0)
    {
      trspistat.leds[0].changed = 1;
      trspistat.leds[0].state = 1;
    }
  else
    {
      trspistat.leds[0].changed = 1;
      trspistat.leds[0].state = 0;
    }
  while (i<100000)
  i++;*/

    i=0;
  
  for (i; i<LOC_NUMLEDS; i++)
    {
      if ((trspistat.leds[i].changed)==1)
	{
	  if ((trspistat.leds[i].state)==0)
	    {
	      trspistat.leds[i].changed = 0;
	      PIO_Clear(&trspistat.leds[i].pioled);
	    }
	  else
	    {
	      trspistat.leds[i].changed = 0;
	      PIO_Set(&trspistat.leds[i].pioled);
	    }
	}
    }
}
