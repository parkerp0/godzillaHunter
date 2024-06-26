#include "open_interface.h"
#include "movement.h"


int main() {

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()



    timer_init();
    lcd_init();

    coords *robotCoords = malloc(sizeof(coords));
    robotCoords->x = 0;
    robotCoords->y = 0;
    robotCoords->heading = 0;

    // test movement
//    turn_right(sensor_data, robotCoords, 405);
//    timer_waitMillis(1000);
//    move_forward(sensor_data, robotCoords, 1000.0);
//    timer_waitMillis(1000);
//    turn_left(sensor_data, robotCoords, 45);
//    timer_waitMillis(1000);
//    move_backward(sensor_data, robotCoords, 1000.0);

    // test move to point
    move_to_point(sensor_data, robotCoords, 610 * 1.5, 610*3);
//    move_to_point(sensor_data, robotCoords, -450, 450);
//    move_to_point(sensor_data, robotCoords, -450, 0);
//    move_to_point(sensor_data, robotCoords, 450, 0);
//    move_to_point(sensor_data, robotCoords, 0, 450);
//    move_to_point(sensor_data, robotCoords, 0, -450);
//    move_to_point(sensor_data, robotCoords, 450, 450);
//    move_to_point(sensor_data, robotCoords, 450, -450);


    oi_setWheels(0,0);

    oi_free(sensor_data);
}


double move_to_point(oi_t *sensor_data, coords *robotCoords, double global_x, double global_y){

    double deltaX = global_x - robotCoords->x;
    double deltaY = global_y - robotCoords->y;
    double targetHeading = fmod(atan2(deltaX, deltaY) * 180.0 / M_PI, 360); // lock in the degrees to be -360 to 360
    double deltaHeading = fabs(robotCoords->heading - targetHeading);

    // Figure out which way to turn
    int turnDir = 1; // default is turn right
    if (targetHeading < robotCoords->heading) // Turn left instead
        turnDir = -1;
//    if (fabs(targetHeading - robotCoords->heading) > 180){ // wraps around so change the direction
//        turnDir *= -1;
//        deltaHeading = 180 - deltaHeading;
//    }

    if (turnDir == 1)
        turn_right(sensor_data, robotCoords, deltaHeading);
    else
        turn_left(sensor_data, robotCoords, deltaHeading);

    lcd_printf("TurnDIR: %d\nTarget: %lf\nCurrent: %lf\nDelta: %lf", turnDir, targetHeading, robotCoords->heading, deltaHeading);

    timer_waitMillis(500);
    double distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    move_forward(sensor_data, robotCoords, distance);

    return 0.0;
}

double move_forward(oi_t *sensor_data, coords *robotCoords, double distance_mm) {
    double sum = 0;
    int power = 10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        double deltaDistance = sensor_data->distance;
        double deltaHeading = -sensor_data->angle;

        robotCoords->heading += deltaHeading;
        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);

        if (distance_mm - sum > power  && power < 200)
            power += 10;
//        else if (sum > distance_mm/2.0  && power > 100)
        else if (distance_mm - sum < power  && power > 20)
            power -= 15;

        if (sensor_data->bumpLeft){
            move_backward(sensor_data, robotCoords,  100.0);
            timer_waitMillis(500);
            turn_right(sensor_data, robotCoords,  45.0);
            timer_waitMillis(500);
            move_forward(sensor_data, robotCoords,  330.0);
            timer_waitMillis(500);
            turn_left(sensor_data, robotCoords,  60.0);
            oi_setWheels(0,0);
            return 0;
        } else if (sensor_data->bumpRight){
            move_backward(sensor_data, robotCoords,  100.0);
            timer_waitMillis(500);
            turn_left(sensor_data, robotCoords,  45.0);
            timer_waitMillis(500);
            move_forward(sensor_data, robotCoords,  330.0);
            timer_waitMillis(500);
            turn_right(sensor_data, robotCoords, 60.0);
            oi_setWheels(0,0);
            return 0;
        }

        oi_setWheels(power-TWISTOFFSET, power + TWISTOFFSET);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }
    oi_setWheels(0,0);
    return sum;
}

double move_backward(oi_t *sensor_data, coords *robotCoords, double distance_mm) {
    double sum = 0;
    int power = -10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum > -distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        double deltaDistance = sensor_data->distance;
        double deltaHeading = -sensor_data->angle;

        robotCoords->heading += deltaHeading;
        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);

        if (distance_mm + sum > -power && power > -200)
            power -= 10;
//        else if (sum > distance_mm/2.0  && power > 100)
        else if (distance_mm + sum < -power  && power < -20)
            power += 15;
//
//        if (sum > -distance_mm/2.0  && power > -200)
//            power -= 10;
//        else if (sum < -distance_mm/2.0  && power < -20)
//            power += 10;

        oi_setWheels(power + TWISTOFFSET, power - TWISTOFFSET);//change this
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }
    oi_setWheels(0,0);
    return sum;
}

double turn_right(oi_t *sensor, coords *robotCoords,  double degrees) {
    double sum = 0;
    oi_setWheels(-200, 200);
    while (sum > -degrees + TURNOFFSET) {//+ 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        double deltaDistance = sensor->distance;
        double deltaHeading = -sensor->angle;

        robotCoords->heading += deltaHeading;
        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }


    robotCoords->heading += TURNOFFSET;
    robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
    lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    oi_setWheels(0,0);
    return sum;
}

double turn_left(oi_t *sensor, coords *robotCoords, double degrees) {
    double sum = 0;
    oi_setWheels(200, -200);
    while (sum < degrees - TURNOFFSET) {//- 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        double deltaDistance = sensor->distance;
        double deltaHeading = -sensor->angle;

        robotCoords->heading += deltaHeading;
        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }


    robotCoords->heading -= TURNOFFSET;
    robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
    lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    oi_setWheels(0,0);
    return sum;
}

double moveCalibrate(oi_t *sensor_data, coords *robotCoords)
{
    lcd_printf("PRESS 4 To move 1 meter");
    while(button_getButton() != 4);
    move_forward(sensor_data, robotCoords, 1000);

    lcd_printf("PRESS 4 To move 2 meters");
    while(button_getButton() != 4);
    move_forward(sensor_data, robotCoords, 2000);
    return 0.0;
}

double turnCalibrate(oi_t *sensor_data, coords *robotCoords)
{

    lcd_printf("PRESS 4 to start turn calibration");
    while(button_getButton() != 4);

    double sum;
    sum = turn_left(sensor_data, robotCoords, 90);
    while(1)
        {
            switch(button_getButton())
            {
            case(1):
                sum+=turn_left(sensor_data, robotCoords, 1);
                break;
            case(2):
                sum+=turn_right(sensor_data, robotCoords, 1);
                break;
            case(3):
                sum+=turn_left(sensor_data, robotCoords, 20);
                break;
            case(4):
                sum+=turn_right(sensor_data, robotCoords, 20);
                break;
            }
            lcd_printf("1: left 1 \n2: right 1 \n3:left 20 \n4:right 20\n%lf", sum);
        }
    return 0.0;
}
