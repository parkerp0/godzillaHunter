#include "servo.h"

#define START_VAL 6920
#define END_VAL   35040

void servo_init()
{
    SYSCTL_RCGCGPIO_R |= 0x2;
    while(!(SYSCTL_RCGCGPIO_R & 0x2));

    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= (~0xF00000);
    GPIO_PORTB_PCTL_R |= 0x700000;

    SYSCTL_RCGCTIMER_R |=0x2;
    while(!(SYSCTL_RCGCTIMER_R & 0x2));

    TIMER1_CTL_R &=(~0x100);
    while(TIMER1_CTL_R & 0x100);

    TIMER1_CFG_R |= 0x4;

    TIMER1_TBMR_R &= (~0xFF);
    TIMER1_TBMR_R |= 0x1A;
    TIMER1_CTL_R |= 0x4000;

    TIMER1_TBPR_R = 0x4;
    TIMER1_TBILR_R = 0xE200;
//    TIMER1_TBMATCHR_R = 0x48440; //1.5 ms away from top of register
    TIMER1_TBMATCHR_R = (END_VAL + START_VAL)/2; //1.5 ms away from top of register

//    TIMER1_TBPR_R = 0x5;
//    TIMER1_TBILR_R = 0x3FC0;
//    TIMER1_TBMATCHR_R = 0x4E200; //20ms from bottom of reg

    TIMER1_CTL_R |= 0x100;
    timer_waitMillis(1000);
    TIMER1_CTL_R &=(~0x100);
}

void servo_move(int degrees)
{
    //min is 16000
    //max is 32000

    TIMER1_CTL_R |= 0x100;
    TIMER1_TBMATCHR_R = (END_VAL - START_VAL)/180 *degrees + START_VAL;
    timer_waitMillis(1000);
    TIMER1_CTL_R &=(~0x100);

}

void servo_calibrate()
{
    char msgBuffer[90];

    button_init();
    while(1)
    {
        switch(button_getButton())
        {
        case(1):
            TIMER1_TBMATCHR_R +=10;
            break;
        case(2):
            TIMER1_TBMATCHR_R -=10;
            break;
        case(3):
            TIMER1_TBMATCHR_R +=100;
            break;
        case(4):
            TIMER1_TBMATCHR_R -=100;
            break;
        }

    sprintf(msgBuffer,"%ud",TIMER1_TBMATCHR_R);
    lcd_printf(msgBuffer);

    }
}
