#include "adc.h"

void adc_init()
{
    SYSCTL_RCGCGPIO_R |= 0x2;
    while(!(SYSCTL_RCGCGPIO_R & 0x2));

    GPIO_PORTB_AFSEL_R |= 0x10;
    GPIO_PORTB_DEN_R &= ~(0x10);
    GPIO_PORTB_AMSEL_R |= 0x10;
    GPIO_PORTB_DIR_R &= ~(0x10);



    SYSCTL_RCGCADC_R |= 0x1;
    while(!(SYSCTL_RCGCADC_R & 0x1));
    ADC0_ACTSS_R &= ~(0x8);
    while(ADC0_ACTSS_R & 0x8);
    ADC0_EMUX_R &= ~(0xF000);
    ADC0_SSMUX3_R |= 0xA;
    ADC0_IM_R &= ~(0x8);
    ADC0_SSCTL3_R |= 0x4;
    ADC0_SAC_R |= 0x6;

    ADC0_ACTSS_R |= 0x8;
}
uint16_t adc_read()
{
    uint16_t temp;

    ADC0_PSSI_R |= 0x8;
    while(!(ADC0_RIS_R & 0x8));
    //timer_waitMillis(3);
    temp = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R |= 0x8;
    return temp;
}
