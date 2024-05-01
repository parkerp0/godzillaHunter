// /*
//  * main.c
//  *
//  *  Created on: Apr 12, 2024
//  *      Author: cdoran
//  */
//
//
//
// //uart interrupt scheme
//
// //scan function with sweep: servo, ping, IR
//
// //functions for bump and cliff sensors
//
// //object classification
//
// //calibration software
//
// //mapping robot and obstacle locations
//
// //ram function
//
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
            robotCoords->x = START_X;
            robotCoords->y = START_Y;

            object *obs = NULL;
            object largest;
            int obsCount = 0;

            int targetX = START_X;
            int targetY = START_Y;

            int i;
            int upFlag;

            char message[90];

            oi_setWheels(0,0);
            command_byte = -1;



            while(1)
            {
                if(command_byte == 'q')
                {
                    command_byte = -1;
                    obsCount = scanAndRewrite(&obs,obsCount);
                    move_to_point(sensorD,&obs,&obsCount,0,obs[0].x*2,obs[0].y*2,1);
                }

                if(command_byte == 't')//start the overall scanning routine
                {
                    command_byte = -1;
                    upFlag = 1;
                    targetX = START_X;
                    obsCount = scanAndRewrite(&obs,obsCount);
                    while(targetX < FIELD_WIDTH)
                    {
                        if(upFlag)targetY = START_Y + 20;
                        else targetY = FIELD_LENGTH - 190;
                        while(targetY < FIELD_LENGTH && targetY > START_Y)
                        {
                            move_to_point(sensorD,&obs,&obsCount,0,targetX,targetY,1);
                            obsCount = scanAndRewrite(&obs,obsCount);
                            if(command_byte == 'b')break;//breaks out after the most recent loop for a restart
                            if(upFlag)targetY+=500;//increment in the correct direction
                            else targetY-=500;

                            if(command_byte == 'b')
                            {
                                command_byte = -1;
                                break;
                            }
                        }

                        if(command_byte == 'b')
                        {
                            command_byte = -1;
                            break;
                        }
                        if(upFlag)upFlag = 0;
                        else upFlag = 1;//flip the up and down
                        targetX+= 500;
                    }
                    //largest = findLargestObject();
                    //move_to_point(sensorD,obs,obsCount,0,largest.x + 60,largest.y + 60);
                    //sprintf(message,"Godzilla is X:%.3f Y:%.3f Width: %.3f\n\r Press k to ram",largest.x,largest.y,largest.linearWidth);
                    //uart_sendStr(message);
                }

                if(command_byte == 'b')//debug block to allow for the putty op to look at the robot state
                {
                    command_byte = -1;
                    sprintf(message,"\n\r\n\r\n\rcurrent location \n\rX: %.3f, \n\rY: %.3f, \n\rHeading: %.3f\n\r",robotCoords->x,robotCoords->y,robotCoords->heading);
                    uart_sendStr(message);
                    for(i = 0; i<obsCount; i++)
                    {
                        sprintf(message,"obsNum:%d X: %.3f, Y: %.3f, Width: %.3f\n\r",i,obs[i].x,obs[i].y,obs[i].linearWidth);
                        uart_sendStr(message);
                    }
                }

                if(command_byte == 'r')//reset the program to allow it to be restarted from putty without having to hardware restart it
                {
                    command_byte = -1;
                    free(obs);
                    obs = NULL;
                    obsCount = 0;
                    robotCoords->heading = 0.0;
                    robotCoords->x = START_X;
                    robotCoords->y = START_Y;
                }

                while(command_byte == 'p')
                {
                    ;//hang here and wait for more input from the terminal
                }
                if(command_byte == 'f')
                {
                    //stop the scanning routine and move toward the zone
                    oi_free(sensorD);
                    free(obs);
                    free(robotCoords);
                    break;
                }

                if(command_byte == 'w')//movement block wasd forward/backward 10 cm left/right 45 degrees
                {
                    command_byte = -1;
                    move_forward(sensorD, &obs, &obsCount, 100, 1, 0);
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
