/**
 * @file main.c
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 *
 * The purpose of this file is to serve as a basic example for IMU usage.
 */

#include "imu.h"
#include "timer.h"
#include "lcd.h"
#include "button.h"
#include "open_interface.h"

void imu_printChipInfo();
void imu_move_distance(float dis, oi_t *sensor_data, int speed);

int main() {
    imu_init();
    lcd_init();
    imu_writeReg(IMU_OPR_MODE, NDOF);
    imu_setDefaultUnits();

    //imu_printChipInfo();


    float heading;
    while(1){
        heading = imu_getHeading();
        //lcd_printf("heading: %f",heading);
        fflush(stdout);
        if(button_getButton() ==4)imu_reset();

        timer_waitMillis(500);   // This is a simple 1 second delay call.
    }

}


/*
 * This method shows a basic algorithm for calculating movement distance using
 * the IMU. OI represents an iRoomba open interface implementation. However,
 * the actual movement operations are trivial. We simply set a speed value to
 * the wheels (two drive wheels, set a speed to each wheel), and then stop the
 * wheels once we have driven our desired distance.
 */
/*
 * This function uses a simple LCD interface. The library functions should
 * be readily apparant. 
 */
void imu_printChipInfo() {
    lcd_init();
    imu_info_t* chipInfo = imu_getChipInfo();

    lcd_printf("Chip ID: %d\n"
            "ACC ID: %d\n"
            "MAG ID: %d\n"
            "GYRO ID: %d\n",
            chipInfo->chipID,
            chipInfo->accID,
            chipInfo->magID,
            chipInfo->gyrID);

    timer_waitMillis(5000);

    lcd_printf("SW Rev: %d\n"
            "BL Rev: %d\n",
            chipInfo->swRevID,
            chipInfo->blRevID);


    timer_waitMillis(5000);
    lcd_clear();
}
