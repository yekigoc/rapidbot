#include "locator.h"
#include "FreeRTOS.h"

unsigned int ConvHex2mV( unsigned int valueToConvert )
{
  return( (ADC_VREF * valueToConvert)/0x3FF);
}
