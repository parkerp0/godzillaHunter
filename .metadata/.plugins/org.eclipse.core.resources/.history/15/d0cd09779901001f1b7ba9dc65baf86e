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
	servo_init();
	ping_init();
	button_init();
	IR_init();

	//moveCalibrate(sensorD);
	//

	//pingCalibrate();
	while(1)
	{
	    switch(button_getButton())
	    {
	    case 0:
	        lcd_printf("1:Move \n2:Servo \n3:Ping \n4:IR");
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
