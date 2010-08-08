#include "FreeRTOS.h"
#include "locator.h"
#include "locator_task.h"
#include "common.h"
#include "task.h"
#include "pio/pio.h"
#include "fft/fix_fft.h"

#define locatorSTACK_SIZE		(1024)
#define SPI_PCS(npcs)       ((~(1 << npcs) & 0xF) << 16)

struct _chanparams
{
  Pin * npcs;
  Pin * cdout;
  Pin * spck;
  unsigned int snpcs;
  char arstate;
  char aroffset;
};
typedef struct _chanparams chanparams;

void vLocatorTask( void *pvParameters );

/*-----------------------------------------------------------*/

void vStartLocatorTask( unsigned portBASE_TYPE uxPriority )
{
/* Spawn the task. */
 xTaskCreate( vLocatorTask,  ( signed portCHAR * ) "Loc", locatorSTACK_SIZE, ( void * ) NULL, uxPriority, ( xTaskHandle * ) NULL );
}
/*-----------------------------------------------------------*/

void loc_writecommand(unsigned char command, unsigned char leavenpcslow, chanparams * p)
{
  portDISABLE_INTERRUPTS();
  PIO_Clear (p->spck);
  int i = 0;
  int j = 0;
  for (i=0; i<100; i++)
    asm("nop");
  PIO_Clear (p->npcs);
  for (i=0; i<100; i++)
    asm("nop");
  for (j=0; j<3; j++)
    {
      PIO_Set (p->spck);
      if ((p->snpcs & 1<<(2-j)))
	PIO_Set (p->cdout);
      else 
	PIO_Clear (p->cdout);
      for (i=0; i<100; i++)
	asm("nop");
      PIO_Clear (p->spck);
      for (i=0; i<100; i++)
	asm("nop");
    }
  for (j=0; j<8; j++)
    {
      PIO_Set (p->spck);
      if ((command & 1<<(7-j)))
	PIO_Set (p->cdout);
      else 
	PIO_Clear (p->cdout);
      for (i=0; i<100; i++)
	asm("nop");
      PIO_Clear (p->spck);
      for (i=0; i<100; i++)
	asm("nop");
    }
  PIO_Set (p->spck);
  for (i=0; i<100; i++)
    asm("nop");
  if (leavenpcslow==0)
    PIO_Set (p->npcs);
  portENABLE_INTERRUPTS();
}

//1 - output 1, 0 - input
void arcontrol(chanparams *p)
{
  Pin ar = {1 << p->aroffset, AT91C_BASE_PIOA, AT91C_ID_PIOA, /*PIO_INPUT*/ PIO_OUTPUT_1, PIO_DEFAULT};
  if (p->arstate == 0)
    {
      ar.type = PIO_INPUT;
    }
  PIO_Configure(&ar, 1);
}

void togglear(chanparams *p)
{
  if (p->arstate == 1)
    p->arstate = 0;
  else
    p->arstate = 1;
  arcontrol(p->arstate);
}


void vLocatorTask( void *pvParameters )
{
  extern void ( vLOC_ISR_Wrapper )( void );
  extern void ( vLOC_TC_ISR_Wrapper )( void );

  struct complex in[FFT_SIZE];

  unsigned int id_channel;
  trspistat.channels[0].channel = ADC_CHANNEL_0;
  trspistat.channels[1].channel = ADC_CHANNEL_1;
  trspistat.channels[2].channel = ADC_CHANNEL_2;
  trspistat.channels[3].channel = ADC_CHANNEL_3;
  trspistat.channels[4].channel = ADC_CHANNEL_4;
  trspistat.channels[5].channel = ADC_CHANNEL_5;
  trspistat.channels[6].channel = ADC_CHANNEL_6;
  trspistat.channels[7].channel = ADC_CHANNEL_7;
  trspistat.processed = 1;

  chanparams locch[8];
  
  int i = 0;
  Pin npcs0 = NPCS0_LOC;
  Pin npcs1 = NPCS1_LOC;
  Pin cdout = CDOUT_LOC;
  Pin spck = SPCK_LOC;

  for (;i<4;i++)
    {
      locch[i].npcs = &npcs0;
      locch[i].cdout = &cdout;
      locch[i].spck = &spck;
      locch[i].snpcs = i;
      locch[i].arstate = 0;
    }

  for (;i<8;i++)
    {
      locch[i].npcs = &npcs1;
      locch[i].cdout = &cdout;
      locch[i].spck = &spck;
      locch[i].snpcs = i;
    }

  i=0;

  locch[0].aroffset = 21;
  locch[1].aroffset = 16;
  locch[2].aroffset = 15;
  locch[3].aroffset = 14;
  locch[4].aroffset = 13;
  locch[5].aroffset = 12;
  locch[6].aroffset = 11;
  locch[5].aroffset = 10;



  unsigned char command = 0x0;

  //------------------------------------------------------------------------------
  /// Configures Timer Counter 0 (TC0) to generate an interrupt every second. This
  /// interrupt will be used to display the number of bytes received on the USART.
  //------------------------------------------------------------------------------
  unsigned int div, tcclks;
  
  // Enable TC0 peripheral clock
  
  if ((AT91C_BASE_PMC->PMC_PCSR & (1 << AT91C_ID_TC0)) == (1 << AT91C_ID_TC0)) 
    {
      
    }
  else 
    { 
      AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;
    }
  
  ADC_Initialize( AT91C_BASE_ADC,
		  AT91C_ID_ADC,
		  AT91C_ADC_TRGEN_DIS,
		  0,
		  AT91C_ADC_SLEEP_NORMAL_MODE,
		  AT91C_ADC_LOWRES_10_BIT,
		  BOARD_MCK,
		  BOARD_ADC_FREQ,
		  20,
		  600);
  

  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_0);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_1);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_2);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_3);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_4);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_5);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_6);
  ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_7);

  AIC_ConfigureIT(AT91C_ID_ADC, 0, vLOC_ISR_Wrapper);

  AIC_EnableIT(AT91C_ID_ADC);

  ADC_EnableIt(AT91C_BASE_ADC,ADC_CHANNEL_0);


  /// Configure TC for a 1s (= 1Hz) tick
  /*  if (TC_FindMckDivisor(390625, BOARD_MCK, &div, &tcclks)==1)
      {*/
  tcclks = 2;
  div = 4;
  TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
  AT91C_BASE_TC0->TC_RC = div;//(BOARD_MCK / (2 * div));
  
  // Configure interrupt on RC compare
  AIC_ConfigureIT(AT91C_ID_TC0, 0, vLOC_TC_ISR_Wrapper);
  AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
  AIC_EnableIT(AT91C_ID_TC0);
  TC_Start(AT91C_BASE_TC0);
      //    }
  
  
  // Start measurement
  unsigned char chan = 0;
  unsigned short max = 0;
  //short x[N];
  int z = 0;
  unsigned long timebefore = 0;
  trspistat.timeafter = 0;
  InitFFTTables();
  trspistat.counter=0;
		  
  int octr = 0;

  // Start measurement
  //  ADC_StartConversion(AT91C_BASE_ADC);

  for(;;)
    {
      if (octr%10 == 0)
	{
	  if (trspistat.leds[0].state == 1)
	    {
	      trspistat.leds[0].state = 0;
	      trspistat.leds[0].changed = 1;
	    }
	  else
	    {
	      trspistat.leds[0].state = 1;
	      trspistat.leds[0].changed = 1;
	    }
	}
      vTaskDelay(5 / portTICK_RATE_MS );
      for (i = 0;i<LOC_NUMADCCHANNELS; i++)
	{
	  if (trspistat.channels[i].ampchanged == 1)
	    {
	      loc_writecommand(trspistat.channels[i].amp-1, 0, &(locch[i]));
	      trspistat.channels[i].ampchanged = 0;
	    }
	}
      if ( trspistat.processed == 0)
	{
	  timebefore = xTaskGetTickCount();	  
	  for (i = 0;i<LOC_NUMADCCHANNELS; i++)
	    {
	      trspistat.channels[i].freqamount = 0;
	      for (z=0;z<LOC_NUMSAMPLES;z++)
		{
		  if (trspistat.channels[i].adcbuf[z]-MIDDLEPOINT>max)
		    max = trspistat.channels[i].adcbuf[z]-MIDDLEPOINT;
		  in[z].r = trspistat.channels[i].adcbuf[z]-MIDDLEPOINT;
		  in[z].i=0;
		}
	      DoFFT(in, FFT_SIZE);
	      for (z=0;z<LOC_NUMSAMPLES;z++)
		{
		  if (z>LEFT_BORDER && z<RIGHT_BORDER)
		    {
		      trspistat.channels[i].freqamount += abs(in[z].r);
		    }
		  trspistat.channels[i].fx[z] = in[z].r;
		}

	      if (trspistat.channels[i].freqamount < 2000)
		{
		  togglear(&(locch[i]));
		}
	      
	      if (max>ALLOWED_MAX)
		{
		  if (trspistat.channels[i].amp>8)
		    {
		      trspistat.channels[i].amp = trspistat.channels[i].amp - 8;
		      trspistat.channels[i].ampchanged = 1;
		    }
		}
	      else if (max<ALLOWED_MIN)
		{
		  if (trspistat.channels[i].amp<248)
		    {
		      trspistat.channels[i].amp = trspistat.channels[i].amp + 8;
		      trspistat.channels[i].ampchanged = 1;
		    }
		}
	      max = 0;
	    }
	  trspistat.timeafter = xTaskGetTickCount();
	  trspistat.counter = trspistat.timeafter - timebefore;
	  trspistat.usbdataready = 1;
	  while (trspistat.usbdataready == 1)
	    {
	      vTaskDelay(1 / portTICK_RATE_MS );
	    }
	  octr ++;
	  trspistat.usbdataready = 0;
	  trspistat.channelconverted = 0;
	  trspistat.processed = 1;
	  ADC_EnableChannel(AT91C_BASE_ADC, trspistat.channelconverted);
	  ADC_EnableIt(AT91C_BASE_ADC, trspistat.channelconverted);
	  ADC_StartConversion(AT91C_BASE_ADC);
	  TC_Start(AT91C_BASE_TC0);
	}
    }
}
