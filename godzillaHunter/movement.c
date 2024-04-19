#include "open_interface.h"
#include "movement.h"


//int main() {
//
//    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
//    oi_init(sensor_data); // do this only once at start of main()
//
//    timer_init();
//    lcd_init();
//
////    int i = 0;
////    for (i=0; i < 4; i++) {
////        move_forward(sensor_data, 500.0);
////        timer_waitMillis(1000);
////        turn_right(sensor_data, 90.0);
////        timer_waitMillis(1000);
////    }
////    turn_left(sensor_data, 90.0);
//
//    move_forward(sensor_data, 2000.0);
//
//    oi_setWheels(0,0);
//
//    oi_free(sensor_data);
//}

double move_forward(oi_t *sensor_data, double distance_mm) {
    double sum = 0;
    int power = 10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum < distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        if (distance_mm - sum > power  && power < 200)
            power += 10;
//        else if (sum > distance_mm/2.0  && power > 100)
        else if (distance_mm - sum < power  && power > 20)
            power -= 15;
/*
        if (sensor_data->bumpLeft){
            move_backward(sensor_data, 100.0);
            timer_waitMillis(500);
            turn_right(sensor_data, 45.0);
            timer_waitMillis(500);
            move_forward(sensor_data, 330.0);
            timer_waitMillis(500);
            turn_left(sensor_data, 60.0);
            oi_setWheels(0,0);
            return 0;
        } else if (sensor_data->bumpRight){
            move_backward(sensor_data, 100.0);
            timer_waitMillis(500);
            turn_left(sensor_data, 45.0);
            timer_waitMillis(500);
            move_forward(sensor_data, 330.0);
            timer_waitMillis(500);
            turn_right(sensor_data, 60.0);
            oi_setWheels(0,0);
            return 0;
        }

        oi_setWheels(power, power);
        lcd_printf("%lf", sum);
    }
        */
    oi_setWheels(0,0);
    return sum;
}

double move_backward(oi_t *sensor_data, double distance_mm) {
    double sum = 0;
    int power = -10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum > -distance_mm) {
        oi_update(sensor_data);
        sum += sensor_data->distance;

        if (sum > -distance_mm/2.0  && power > -400)
            power -= 10;
        else if (sum < -distance_mm/2.0  && power < -10)
            power += 10;

        oi_setWheels(power, power);
        lcd_printf("%lf", sum);
    }
    oi_setWheels(0,0);
    return sum;
}


double turn_right(oi_t *sensor, double degrees) {
    double sum = 0;
    oi_setWheels(-200, 200);
    while (sum > -degrees + TURNOFFSET) {//+ 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        lcd_printf("%lf", sum);
    }
    oi_setWheels(0,0);
    return sum;
}

double turn_left(oi_t *sensor, double degrees) {
    double sum = 0;
    oi_setWheels(200, -200);
    while (sum < degrees - TURNOFFSET) {//- 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        lcd_printf("%lf", sum);
    }
    oi_setWheels(0,0);
    return sum;
}

void manuever(oi_t *sensor_data, float distance_mm){
    float distance = 0;
    oi_update(sensor_data);

    while (distance < distance_mm){
        oi_update(sensor_data);
        distance += move_forward(sensor_data, distance);

        //cases for bumping either left or right
        if (sensor_data->bumpLeft && sensor_data->bumpRight){
            oi_update(sensor_data);
            move_backward(sensor_data, 100);
            turn_right(sensor_data, 90);
            distance += move_forward(sensor_data, 250);
        }

        //case for bumping left
        else if(sensor_data->bumpLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, 100);
            turn_right(sensor_data, 90);
            distance += move_forward(sensor_data, 250);
            turn_left(sensor_data, 90);
        }

        //case for bumping right
        else if(sensor_data->bumpRight){
            oi_update(sensor_data);
            move_backward(sensor_data, 100);
            turn_left(sensor_data, 90);
            distance += move_forward(sensor_data, 250);
            turn_right(sensor_data, 90);
        }

        //case for hitting a cliff on the left
        else if(sensor_data->cliffLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, 50);
            turn_right(sensor_data, 90);
            distance += move_forward(sensor_data, 300);
            turn_left(sensor_data, 90);
        }

        //case for hitting a cliff on the right
        else if(sensor_data->cliffRight){
            oi_update(sensor_data);
            move_backward(sensor_data, 50);
            turn_left(sensor_data, 90);
            distance += move_forward(sensor_data, 300);
            turn_right(sensor_data, 90);
        }

        //case for hitting a cliff on the front left
        else if(sensor_data->cliffFrontLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, 50);
            turn_right(sensor_data, 90);
            distance += move_forward(sensor_data, 300);
        }

        //case for hitting the boundary
        //else if(sensor_data->cliffFrontLeftSignal > 2600)
    }
}

