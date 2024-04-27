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
 #include "movement.h"
 #include "scan.h"
 #include "open_interface.h"
 #include "IR.h"
 #include "lcd.h"
 #include "uart-interrupt.h"
 #include "servo.h"
 #include "ping.h"
 #include "button.h"
 #include <inc/tm4c123gh6pm.h>
#include "structs.h"


 int main (void) {

             oi_t *sensorD = oi_alloc();
             oi_init(sensorD);

             timer_init();
             lcd_init();
             uart_interrupt_init();
             IR_init();
             ping_init();
             servo_init();
             button_init();
   
            coords *robotCoords = malloc(sizeof(coords));
            robotCoords->heading = 0.0;
            robotCoords->x = 0.0;
            robotCoords->y = 0.0;

            object *obs = NULL;
            int obsCount = 0;

            while(1)
            {
                if(command_byte == 'r')
                {
                    command_byte = -1;
                    obsCount = scanAndRewrite(&obs,obsCount);
                }
                //find a way to iterate through current obs and check for new obs in scan

                while(command_byte == 't')
                {
                    ;//hang here and wait for more input from the terminal
                }
                while(command_byte == 'f')
                {
                    //stop the scanning routine and move toward the zone
                }

                if(command_byte == 'w')//movement block wasd forward/backward 10 cm left/right 45 degrees
                {
                    command_byte = -1;
                    move_forward(sensorD, 100);
                }
                if(command_byte == 'a')
                {
                    command_byte = -1;
                    turn_left(sensorD,45);
                }
                if(command_byte == 's')
                {
                    command_byte = -1;
                    move_backward(sensorD,100);
                }
                if(command_byte == 'd')
                {
                    command_byte = -1;
                    turn_right(sensorD,45);
                }
                if(command_byte == 'k')
                {
                    ram(sensorD);
                    break;
                }
            }
 }
