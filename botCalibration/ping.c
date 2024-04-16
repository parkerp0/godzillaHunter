/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"
#include "stdlib.h"

volatile unsigned long START_TIME = 0xFFFF;
volatile unsigned long END_TIME = 0;
volatile enum{LOW, HIGH, DONE} STATE = LOW; // State of ping echo pulse

void ping_init (void){

  // YOUR CODE HERE

    SYSCTL_RCGCGPIO_R |= 0x2;
    while(!(SYSCTL_RCGCGPIO_R & 0x2));

    GPIO_PORTB_AFSEL_R |= 0x8;
    GPIO_PORTB_PCTL_R &= (~0xF000);
    GPIO_PORTB_PCTL_R |= 0x7000;
    GPIO_PORTB_DEN_R |= 0x8;

    SYSCTL_RCGCTIMER_R |=0x8;
    while(!(SYSCTL_RCGCTIMER_R & 0x8));

    // Configure and enable the timer
    TIMER3_CTL_R &=(~0x100);
    while(TIMER3_CTL_R & 0x100);

    TIMER3_CFG_R |= 0x4;
    TIMER3_TBMR_R &= (~0x10);
    TIMER3_TBMR_R |= 0x7;
    TIMER3_CTL_R |= 0xc00;
    TIMER3_TBPR_R |= 0xFF;
    TIMER3_TBILR_R |= 0xFFFF;
    TIMER3_IMR_R |= 0x400;


    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF1F) | 0x00000020;

    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    IntMasterEnable();

    TIMER3_CTL_R |= 0x100;
    while(!(TIMER3_CTL_R & 0x100));
}

void ping_trigger (void){
    STATE = LOW;
    // Disable timer and disable timer interrupt
    TIMER3_CTL_R &= (~0x100);
    TIMER3_IMR_R &= (~0x400);

    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &=(~0x8);
    GPIO_PORTB_DIR_R |= 0x8;

    GPIO_PORTB_DATA_R &= (~0x8);
    GPIO_PORTB_DATA_R |=0x8;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= (~0x8);

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R |= 0x400;
    // Re-enable alternate function, timer interrupt, and timer

    GPIO_PORTB_DIR_R &= (~0x8);
    GPIO_PORTB_AFSEL_R |= 0x8;
    TIMER3_IMR_R |= 0x400;
    TIMER3_CTL_R |= 0x100;
    while(!(TIMER3_CTL_R & 0x100));
}

void TIMER3B_Handler(void){

  // YOUR CODE HERE
  // As needed, go back to review your interrupt handler code for the UART lab.
  // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
  // includes checking the source of the interrupt and clearing the interrupt status bit.
  // Checking the source: test the MIS bit in the MIS register (is the ISR executing
  // because the input capture event happened and interrupts were enabled for that event?
  // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
  // The rest of the code in the ISR depends on actions needed when the event happens.

  if(TIMER3_MIS_R & 0x400)
  {
      TIMER3_ICR_R |=0x400;
      if(STATE == LOW)
      {
          START_TIME = TIMER3_TBR_R;
          STATE = HIGH;
      }else if(STATE == HIGH)
      {
          END_TIME = TIMER3_TBR_R;
          STATE = DONE;
      }
  }



}

float ping_getDistance (void){
    unsigned long dif;
    //char message[100];

    ping_trigger();
    while(STATE != DONE);
    if(START_TIME>END_TIME)
        dif = START_TIME - END_TIME;
    else
        dif = START_TIME + (0xFFFFFF - END_TIME);

    //sprintf(message, "THIS IS A TEST");
    //lcd_printf(message);

    return dif * 0.0000107 + PINGOFFSET;//scaled to meters

}

unsigned long ping_getPulseWidth(){
    unsigned long width;
    char message[90];

    ping_trigger();
        while(STATE != DONE);
        if(START_TIME>END_TIME)
        {
            width = START_TIME - END_TIME;
            sprintf(message, "pulse Width: %lu \nDist: %f \nNo rollover",width,width* 0.0000107 + PINGOFFSET);
        }
        else
        {
            width = START_TIME + (0xFFFFFF - END_TIME);
            sprintf(message, "pulse Width: %lu \nDist: %f \n rollover",width,width* 0.0000107 + PINGOFFSET);
        }


        lcd_printf(message);
        return width;
}

float pingCalibrate()
{
    unsigned long width;

        ping_trigger();


    while(STATE != DONE);
            if(START_TIME>END_TIME)
            {
                width = START_TIME - END_TIME;
            }
            else
            {
                width = START_TIME + (0xFFFFFF - END_TIME);
            }


            lcd_printf("width: %lu dist: %f",width,width* 0.0000107);
            return width;
}
