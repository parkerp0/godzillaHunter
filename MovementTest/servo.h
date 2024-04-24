#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdlib.h>
#include "button.h"
#include "lcd.h"
#include "timer.h"

void servo_init();

void servo_move(int degrees);

void servo_calibrate();
