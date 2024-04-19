#include <stdint.h>
#include <stdlib.h>
#include "movement.h"
#include <math.h>
#include <inc/tm4c123gh6pm.h>
#include "button.h"
#include "lcd.h"

#define IROFFSET 1 //measure against wall look for linear error of IR sensor

void IR_init();
uint16_t IR_read();
float IR_dist();
float IR_calibrate(oi_t *sensor_data);
