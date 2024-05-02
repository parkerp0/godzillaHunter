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

    oi_setWheels(0,0);

	timer_init();
	lcd_init();
	servo_init();
	ping_init();
	button_init();
	IR_init();

	//moveCalibrate(sensorD);
	//

	oi_setMotorCalibration(1.0,15);
	//pingCalibrate();
	while(1)
	{


	    switch(button_getButton())
	    {
	    case 0:
	        lcd_printf("oi_calLeft: %lf \noi_calRight %lf",oi_getMotorCalibrationLeft(),oi_getMotorCalibrationRight());

	    break;
	    case 1:
	        moveCalibrate(sensorD);
	        timer_waitMillis(1000);
	        turnCalibrate(sensorD);
	        timer_waitMillis(1000);
	    break;
	    case 2:
	        servo_calibrate();
	        timer_waitMillis(1000);
	    break;
	    case 3:
	        pingCalibrate();
	        timer_waitMillis(1000);
	    break;
	    case 4:
	        IR_calibrate(sensorD);
	        timer_waitMillis(1000);
	    break;

	    }

	}


	//servo_calibrate();

	oi_free(sensorD);
}
