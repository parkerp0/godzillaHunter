/*
 * scan.h
 *
 *  Created on: Apr 16, 2024
 *      Author: cdoran
 */
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"
#include "open_interface.h"
#include "IR.h"
#include "lcd.h"
#include "uart-interrupt.h"
#include "servo.h"
#include "ping.h"
#include <stdlib.h>
#include <math.h>

#ifndef SCAN_H_
#define SCAN_H_

#define degreesToRadians M_PI/180.0

typedef struct
{
    double x;
    double y;
    double linearWidth;
}object;

object* scan();

#endif /* SCAN_H_ */
