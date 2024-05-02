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
#include "open_interface.h"

void imu_printChipInfo();
void imu_move_distance(float dis, oi_t *sensor_data, int speed);

int main() {
    timer_init();
    i2c_init();
    imu_writeReg(IMU_OPR_MODE, NDOF);
    imu_setDefaultUnits();

    imu_printChipInfo();


    mag_t* mag;
    float heading;
    while(1){
        mag = imu_getMag();
        heading = mag->heading;
        lcd_printf("heading: %f",heading);
        free(mag);
        fflush(stdout);

        timer_waitMillis(1000);   // This is a simple 1 second delay call. 
    }

}


/*
 * This method shows a basic algorithm for calculating movement distance using
 * the IMU. OI represents an iRoomba open interface implementation. However,
 * the actual movement operations are trivial. We simply set a speed value to
 * the wheels (two drive wheels, set a speed to each wheel), and then stop the
 * wheels once we have driven our desired distance.
 */
void imu_move_distance(float dis, oi_t *sensor_data, int speed){
    if (!sensor_data) {
        return;
    }
    float v = 0, x = 0;
    int t = timer_getMicros();  // This is a library call to a running on board timer. Gets current time in micro-Seconds
    int tLast;
    acc_t* acc;
    int direction = (dis > 0) * 2 - 1;
    dis = abs(dis);


    oi_setWheels(speed * direction, speed * direction);
    tLast = t;
    while (x < dis) {
        acc = imu_getAcc();
        t = timer_getMicros();
        v = v + (acc->magnitude * (t/1000000.0f - tLast/1000000.0f));
        x = x + (v * (t/10000.0f - tLast/10000.0f));
        free(acc);
        tLast = t;
    }
    oi_setWheels(0, 0);
}

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
