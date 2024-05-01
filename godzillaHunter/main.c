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

coords *robotCoords;

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
   
            robotCoords = malloc(sizeof(coords));
            robotCoords->heading = 0.0;
            robotCoords->x = 0.0;
            robotCoords->y = 0.0;

            object *obs = NULL;
            object largestObject;
            object *largestObj;
            int obsCount = 0;

            while(1)
            {
                if(command_byte == 'r')
                {
                    command_byte = -1;
                    obsCount = scanAndRewrite(&obs,obsCount);
                    move_to_point(sensorD,obs,obsCount,0,obs[0].x * 2, obs[0].y * 2);
                    turn_left(sensorD,180);
                    obsCount = scanAndRewrite(&obs,obsCount);
                    //largestObj = realFindLargestObj(&obs,obsCount);
                }

                if(command_byte == 'l')
                {
                    //find the largest object in a scan
                    command_byte = -1;
                    obsCount = scanAndRewrite(&obs,obsCount);
                    obsCount = scanAndRewrite(&obs,obsCount);
                    largestObj = realFindLargestObj(&obs,obsCount);
                }

                if(command_byte == 'b')//debug block to allow for the putty op to look at the robot state
                {
                    command_byte = -1;
                    sprintf(message,"\n\r\n\rcurrent location \n\rX: %.3f, \n\rY: %.3f, \n\rHeading: %.3f\n\r",robotCoords->x,robotCoords->y,robotCoords->heading);
                    uart_sendStr(message);
                    for(i = 0; i<obsCount; i++)
                    {
                        sprintf(message,"obsNum:%d X: %.3f, Y: %.3f, Width: %.3f\n\r",i,obs[i].x,obs[i].y,obs[i].linearWidth);
                        uart_sendStr(message);
                    }
                }

                //find a way to iterate through current obs and check for new obs in scan

                if(command_byte == 't')//start the overall scanning routine
                {
                    command_byte = -1;

                    while(targetX < FIELD_WIDTH)
                    {
                        while(targetY < FIELD_LENGTH)
                        {
                            obsCount = scanAndRewrite(&obs,obsCount);
                            if(command_byte == 'b')break;//breaks out after the most recent loop for a restart
                            targetY += 500;
                            move_to_point(sensorD,obs,&obsCount,0,targetX,targetY,1);

                        }
                        if(command_byte == 'b')
                        {
                            command_byte = -1;
                            break;
                        }
                        targetX+= 500;
                    }
                    //largest = findLargestObject();
                    //move_to_point(sensorD,obs,obsCount,0,largest.x + 60,largest.y + 60);
                    //sprintf(message,"Godzilla is X:%.3f Y:%.3f Width: %.3f\n\r Press k to ram",largest.x,largest.y,largest.linearWidth);
                    //uart_sendStr(message);
                }
             
                if(command_byte == 'f')
                {
                    //stop the scanning routine and move toward the zone
                    oi_free(sensorD);
                    free(obs);
                    free(robotCoords);
                    break;
                }

                if(command_byte == 'h')
                {
                    command_byte = -1;
                    play_victoryChant();
                    oi_update(sensorD);
                }
                while(command_byte == 'p')
                {
                    ;//hang here and wait for more input from the terminal
                }

                if(command_byte == 'w')//movement block wasd forward/backward 10 cm left/right 45 degrees
                {
                    command_byte = -1;
                    //move_forward(sensorD, obs, &obsCount, 500, 1);
                    manuever(sensorD, 500.0);
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
                    oi_update(sensorD);
                    break;
                }
            }
 }
