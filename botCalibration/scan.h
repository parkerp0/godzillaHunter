/*
 * scan.h
 *
 *  Created on: Apr 16, 2024
 *      Author: cdoran
 */

#ifndef SCAN_H_
#define SCAN_H_

#include <inc/tm4c123gh6pm.h>
#include "Timer.h"
#include "open_interface.h"
#include "IR.h"
#include "lcd.h"
#include "uart-interrupt.h"
#include "movement.h"
#include "servo.h"
#include "ping.h"
#include "structs.h"
#include <stdlib.h>
#include <math.h>

#define degreesToRadians M_PI/180.0
#define radianToDegrees 180.0/M_PI
#define SERVO_CENTER_OFFSET 126.5 // distance from the center of the servo to the center of the robot in mm
#define IR_SERVO_OFFSET 38.1 // distance from the outward face of the IR sensor to the center of the servo in mm

#define tMaxLW 170.0 //MAX target linear width
#define tMinLW 130.0 //MIN target linear width

object* scan();
object* findLargestObj(object **currentObs, int obsCount);

int scanAndRewrite(object **currentObs,int obsCount);

float vectorDifMag(object *obs,object *obs2);

object* detectGodzilla(object **currentObs, int obsCount);

bool confirmGodzilla();

#endif /* SCAN_H_ */
