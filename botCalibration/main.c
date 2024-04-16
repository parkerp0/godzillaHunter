#include "IR.h"
#include "lcd.h"
#include "movement.h"
#include "ping.h"
#include "servo.h"
#include "Timer.h"
#include "button.h"

/**
 * main.c
 */
int main(void)
{
    oi_t *sensorD = oi_alloc();
    oi_init(sensorD);

	timer_init();
	lcd_init();
	ping_init();
	servo_init();
	button_init();

	//moveCalibrate(sensorD);
	//turnCalibrate(sensorD);

	while(1)pingCalibrate();


	//servo_calibrate();

	oi_free(sensorD);
}
