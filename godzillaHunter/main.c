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
#include "IR.h"
#include "lcd.h"
#include "uart-interrupt.h"
#include "servo.h"
#include "ping.h"
#include "button.h"
#include <inc/tm4c123gh6pm.h>

#define objMatchThresh 0.03

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

            coords *coord = malloc(sizeof(coords));
            coord->heading = 0.0;
            coord->x = 0.0;
            coord->y = 0.0;

            object *obs = NULL;
            object *obsTemp = NULL;
            int obsCount = 0;
            int i;
            int flag =1;

            char message[90];

            //oi_setWheels(0,0);


            while(1)
            {
                if(command_byte == 'r')
                {
                    command_byte = -1;
                    obsTemp = scan();
                    while(obsTemp->linearWidth!= 0.0)
                    {
                        for(i = 0; i<obsCount; i++)
                        {
                            if(((obsTemp->x < (obs[i].x + objMatchThresh)) && (obsTemp->x > (obs[i].x - objMatchThresh))) && (obsTemp->y < (obs[i].y + objMatchThresh) && (obsTemp->y > (obs[i].y - objMatchThresh))))
                            {//rewrite for making the if statement work
                                obs[i].linearWidth = (obs[i].linearWidth + obsTemp->linearWidth)/2;
                            }else
                                flag = 1;
                        }
                        if(flag)
                        {
                            obsCount++;
                            obs = realloc(obs,sizeof(object)*obsCount);
                            //obsCopy(obs[obsCount-1],obsTemp);
                            obs[obsCount-1].x = obsTemp->x;
                            obs[obsCount-1].y = obsTemp->y;
                            obs[obsCount-1].linearWidth = obsTemp->linearWidth;
                            flag = 0;
                        }
                        
                    obsTemp++;
                    }
                    for(i = 0; i<obsCount; i++)
                    {
                        sprintf(message,"obs %d: x: %.2f y:%.2f Width:%.2f\n\r",i,obs[i].x,obs[i].y,obs[i].linearWidth);
                        uart_sendStr(message);
                    }//prints all objects
                    
                    sprintf(message,"Hunter location x:%lf y:%lf heading: %lf\n\r", coord->x,coord->y,coord->heading);
                    uart_sendStr(message);
                    free(obsTemp);
                    obsTemp = NULL;
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
                    move_forward(sensorD,coord, 100);
                }
                if(command_byte == 'a')
                {
                    command_byte = -1;
                    turn_left(sensorD,coord,45);
                }
                if(command_byte == 's')
                {
                    command_byte = -1;
                    move_backward(sensorD,coord,100);
                }
                if(command_byte == 'd')
                {
                    command_byte = -1;
                    turn_right(sensorD,coord,45);
                }
                if(command_byte == 'k')
                {
                    ram(sensorD,coord);
                    break;
                }
            }


}
