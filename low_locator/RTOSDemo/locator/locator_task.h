#ifndef __LOCATOR_TASK_H__
#define __LOCATOR_TASK_H__

#include <adc/adc.h>
#include <tc/tc.h>

typedef struct _locatorstat
{
  char state;
  unsigned short data[128];
  unsigned short position;
} locatorstat;

locatorstat locstat;

void vStartLocatorTask( unsigned portBASE_TYPE uxPriority );

#endif
