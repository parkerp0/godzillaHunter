#include "IR.h"
#include "lcd.h"
#include "movement.h"
#include "ping.h"
#include "servo.h"
#include "Timer.h"
#include "button.h"
#include "uart-interrupt.h"
#include "bno055.h"

/**
 * main.c
 */

bno_calib_t *bno_calib = 0x0;
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
	uart_interrupt_init();

	bno_t *bno = bno_alloc();
	bno_initCalib(bno_calib);

	char buffer[90];


	while(1)
	{
	    switch(button_getButton())
	    {
	    case 0:
	        lcd_printf("1:Move \n2:Servo \n3:Ping \n4:IMU out");
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
	        bno_calibrateInteractive();
	        while(1)
	        {
	            bno_update(bno);
	            sprintf(buffer,"HEADING: %d, ROLL %d, PITCH: %d \r\nHEADING: %d, ROLL %d, PITCH: %d",
	                    bno->euler.heading, bno->euler.roll, bno->euler.pitch,
	                    bno->euler.heading /16., bno->euler.roll /16., bno->euler.pitch/16.);
	            uart_sendStr(buffer);
	            timer_waitMillis(100);
	        }
	    break;

	    }

	}


	//servo_calibrate();

	oi_free(sensorD);
}
