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
#include "imu.h"

coords *robotCoords;
oi_t *sensorD;

 int main (void) {

             sensorD = oi_alloc();
             oi_init(sensorD);

             timer_init();
             lcd_init();
             uart_interrupt_init();
             IR_init();
             ping_init();
             servo_init();
             button_init();
             imu_init();

            robotCoords = malloc(sizeof(coords));
            robotCoords->heading = 0.0;
            robotCoords->x = START_X;
            robotCoords->y = START_Y;

            object *obs = malloc(0);
            object *largest;
            int obsCount = 0;

            int targetX = START_X;
            int targetY = START_Y;

            prevX = 0;
            prevY = 0;

            int i;
            int upFlag;

            char message[90];

            oi_setWheels(0,0);
            command_byte = 0;

            float Eheading;

            while(1)
            {
                if(command_byte == 'i')
                {
                    command_byte = 0;
                    Eheading = imu_getHeading();
                    //set_heading(sensorD,0);
                }

                if(command_byte == 'q')
                {
                    command_byte = 0;
//                    obsCount = scanAndRewrite(&obs,obsCount);
//                    move_to_point(sensorD,&obs,&obsCount,0,obs[0].x*2,obs[0].y*2,1);
                    set_heading(sensorD, -135);
                }

                if(command_byte == 'g')
                {
                    command_byte = 0;
                    obsCount = scanAndRewrite(&obs,obsCount);

                    move_to_godzilla(sensorD, obs, &obsCount, &obs[0], 1);
                }

                if(command_byte == 't')//start the overall scanning routine
                {
                    command_byte = 0;
                    upFlag = 1;
                    targetX = START_X;
//                    obsCount = scanAndRewrite(&obs,obsCount);
                    while(targetX < FIELD_WIDTH)
                    {
                        if(upFlag)targetY = START_Y + 20;
                        else targetY = FIELD_LENGTH - 20;
                        while(targetY < FIELD_LENGTH && targetY > START_Y)
                        {
                            robotCoords->heading = imu_getHeading();
                            move_to_point(sensorD,&obs,&obsCount,0,targetX,targetY,1);
                            set_heading(sensorD, upFlag ? 0 : 180);
                            obsCount = scanAndRewrite(&obs,obsCount);
                            if(command_byte == 'b')break;//breaks out after the most recent loop for a restart
                            if(upFlag)targetY+=500;//increment in the correct direction
                            else targetY-=500;

                            if(command_byte == 'b')
                            {
                                //command_byte = 0;can't reset command byte here because it doesn't break the second loop if you do
                                break;
                            }
                        }

                        if(command_byte == 'b')
                        {
                            command_byte = 0;
                            break;
                        }

                        turn_right(sensorD, 180);
                        if(upFlag)upFlag = 0;
                        else upFlag = 1;//flip the up and down
                        targetX+= 500;
                    }
                    sprintf(message,"ENDED MAIN LOOP\n\r");
                    uart_sendStr(message);

                    largest = findLargestObj(&obs, obsCount);
                    //oi_t *sensor_data, object *obs, int *numObs, object *godzilla, int dir
                    move_to_godzilla(sensorD,obs,&obsCount,largest, 1);
                    sprintf(message,"Godzilla is X:%.3f Y:%.3f Width: %.3f\n\r Press k to ram",largest->x,largest->y,largest->linearWidth);
                    uart_sendStr(message);
                }

                if(command_byte == 'b')//debug block to allow for the putty op to look at the robot state
                {
                    command_byte = 0;
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
                    command_byte = 0;
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
                    command_byte = 0;
                    move_forward(sensorD, &obs, &obsCount, 100, 1, 0);
                }
                if(command_byte == 'a')
                {
                    command_byte = 0;
                    turn_left(sensorD,45);
                }
                if(command_byte == 's')
                {
                    command_byte = 0;
                    move_backward(sensorD,100);
                }
                if(command_byte == 'd')
                {
                    command_byte = 0;
                    turn_right(sensorD,45);
                }
                if(command_byte == 'k')
                {
                    ram(sensorD);
                    break;
                }
            }
}
