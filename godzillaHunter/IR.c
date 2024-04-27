#include "IR.h"

uint16_t *lookupIR;
float *lookupDist;


void IR_init()
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
uint16_t IR_read()
{
    uint16_t temp;

    ADC0_PSSI_R |= 0x8;
    while(!(ADC0_RIS_R & 0x8));
    //timer_waitMillis(3);
    temp = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R |= 0x8;
    return temp;
}

float IR_dist()
{
    int i;
    //return 8057.6 * pow(IR_read(),-0.937) * IROFFSET;
    uint16_t data = IR_read();
    for(i =0; i<20; i++)
    {
        if(data > lookupIR[i])
        {
            return (1.0*data/lookupIR[i])*lookupDist[i];
        }
    }
    return 1.0*data/lookupIR[19]*lookupDist[19];
}

float IR_calibrate(oi_t *sensor_data)
{
    int i;
    lookupIR = malloc(sizeof(uint16_t)*20);
    lookupDist = malloc(sizeof(float)*20);
    lookupDist[0] = 100.0;
    for(i = 0; i<20; i+=1)
    {
    lookupIR[i] = IR_read();
    if(i !=19)lookupDist[i+1] = lookupDist[i] - move_backward(sensor_data, 50);
    timer_waitMillis(50);
    }

    while(button_getButton()!= 4)
    {
    lcd_printf("Press 4 to exit\n%f",IR_dist());
    timer_waitMillis(5);
    }
    return 0.0;
}

