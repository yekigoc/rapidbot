#ifndef __COMMON_STRUCTS_H__
#define __COMMON_STRUCTS_H__

#define USB_PWM_W    0x02
#define USB_TIME_R   0x03
#define USB_ADC_R    0x05
#define USB_LOC_PWM_W 0xC

typedef struct _pwmparams
{
  unsigned short dutycycles;
  unsigned char cyclechange;
} pwmparams;


#endif //__COMMON_STRUCTS_H__
