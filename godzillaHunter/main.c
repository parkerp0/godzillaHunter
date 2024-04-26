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

coords *robotCoords = NULL;

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


		     int numObs = 3;
             object *obs =  malloc(sizeof(object) * numObs);
             object *obsTemp = NULL;

             obs[0].x = 100000; // mm
             obs[0].y = 100000;
             obs[0].linearWidth = 2.54*4; // about 4 inches wide (in mm)


             obs[1].x = 100000; // mm
             obs[1].y = 100000;
             obs[1].linearWidth = 2.54*4; // about 4 inches wide (in mm)



             obs[2].x = 10000; // mm
             obs[2].y = 10000;
             obs[2].linearWidth = 2.54*4; // about 4 inches wide (in mm)

             robotCoords = malloc(sizeof(coords));
             robotCoords->x = 0;
             robotCoords->y = 0;
             robotCoords->heading = 0;

             oi_setWheels(0,0);

			 lcd_printf("Move To Point test");
			 uart_sendStr("-----------------------------Move to Point test--------------------------------\n\r");
 //            while(button_getButton()!=4);
 //            {
 //                ram(sensorD);
 //            }

//             while(1)
//             {
//                 obs = scan(robotCoords);

                 //find a way to iterate through current obs and check for new obs in scan
//             }

//			 move_to_point(sensorD, obsTemp, 3, 0, 0, 1000);

			 while (1){
				 cliff_detected(sensorD);
			 }


 }
