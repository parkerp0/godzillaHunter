#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>

void adc_init();
uint16_t adc_read();

#endif
