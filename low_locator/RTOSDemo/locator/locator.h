#include "common.h"

#define BOARD_ADC_FREQ 5000000
#define ADC_VREF       3300  // 3.3 * 1000
#define FFT_SIZE  LOC_NUMSAMPLES
#define log2FFT   6
#define N         (2 * FFT_SIZE)
#define log2N     (log2FFT + 1)

unsigned int
ConvHex2mV( unsigned int valueToConvert );

