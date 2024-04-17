/*
 * main.c
 *
 *  Created on: Apr 12, 2024
 *      Author: cdoran
 */



//uart interrupt scheme

//scan function with sweep: servo, ping, IR

//functions for bump and cliff sensors

//object classification

//calibration software

//mapping robot and obstacle locations

//ram function
#include "Timer.h"
#include "scan.h"
#include "open_interface.h"
#include "adc.h"
#include "lcd.h"
#include "uart-interrupt.h"
#include "servo.h"
#include "ping.h"
#include "button.h"
#include <inc/tm4c123gh6pm.h>



int main (void) {
            timer_init();
            lcd_init();
            uart_interrupt_init();
            adc_init();
            ping_init();
            servo_init();


            scan();

}
