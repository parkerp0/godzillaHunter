#include "movement.h"



int main (void) {

            oi_t *sensorD = oi_alloc();
            oi_init(sensorD);

            timer_init();
            lcd_init();
            uart_interrupt_init();
            IR_init();
            ping_init();
            servo_init();
            button_init();

            object *obs = malloc(sizeof(object) * 2);
            // object *obsTemp = NULL;
            int obsCount;

            coords *robotCoords = malloc(sizeof(coords));
            robotCoords->x = 0;
            robotCoords->y = 0;
            robotCoords->heading = 0;

            oi_setWheels(0,0);

            obs[1].x = 500; // mm
            obs[1].y = 500;
            obs[1].linearWidth = 2.54*4; // about 4 inches wide (in mm)

            lcd_printf("Move To Point test");
            move_to_point(sensorD, robotCoords, obs, 1000, 1000);

            oi_setWheels(0,0);

}



double move_to_point(oi_t *sensor_data, coords *robotCoords, object *obs, double global_x, double global_y){
	// TODO check if the point is outside of the field.

    checkObstacles(sensor_data, robotCoords, obs, global_x, global_y);

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

double checkObstacles(oi_t *sensor_data, coords *robotCoords, object *obs, double global_x, double global_y){
    int numObs = sizeof(obs) / sizeof(object);

    // TODO check if the target point is inside of an obstacle zone

    // Sort obstacles based on distances to the robot
    qsort(obs, numObs, sizeof(object), compareDistances);

    // Loop through sorted obstacles
    for (int i = 0; i < numObs; i++) {
        // Calculate the vector from the robot to the target
        double targetVectorX = global_x - robotCoords->x;
        double targetVectorY = global_y - robotCoords->y;

        // Calculate the vector from the robot to the obstacle
        double obstacleVectorX = obs[i].x - robotCoords->x;
        double obstacleVectorY = obs[i].y - robotCoords->y;

        // Calculate the dot product of the two vectors
        double dotProduct = targetVectorX * obstacleVectorX + targetVectorY * obstacleVectorY;

        // Check if the obstacle is between the robot and the target
        // Greater than 0 ensures that the obstacle is not behind or to the side of the robot.
        // the targetVector part ensures that the obstacle is not past the target vector.
        if (dotProduct > 0 && dotProduct < (targetVectorX * targetVectorX + targetVectorY * targetVectorY)) {
            // Obstacle lies between robot and target, calculate distPath
            if (obs[i] == NULL) return 0.0;

            float distPath = calcDistToPath(robotCoords, &obs[i], global_x, global_y);
            if ((distPath - (obs[i].linearWidth / 2.0) - (ROBOT_WIDTH / 2.0)) <= AVOID_DISTANCE) {
                // Avoid obstacle

                coords newTarget = calculatePerpendicularPoint(robotCoords, obs[i]);

                // Recursively avoid each object in the path.
                move_to_point(sensor_data, robotCoords, obs, newTarget.x, newTarget.y);
                

            }
        }
    }

    return 0.0;
}


// Define your comparison function for qsort
int compareDistances(const void *a, const void *b) {
    object *obsA = (object *)a;
    object *obsB = (object *)b;
    // Calculate distances to the robot for both objects
    double distA = calcDistToRobot(&robotCoords, obsA);
    double distB = calcDistToRobot(&robotCoords, obsB);
    // Compare distances
    if (distA < distB) return -1;
    else if (distA > distB) return 1;
    else return 0;
}

double calcDistToRobot(coords *robotCoords, object *obs){
    if (obs == NULL) return 1; // make null values be last
	return sqrt(((obs->x-robotCoords->x)*(obs->x-robotCoords->x)) + ((obs->y-robotCoords->y)*(obs->y-robotCoords->y)));
}

coords calculatePerpendicularPoint(coords robotCoords, object targetCoords) {
    // Calculate direction vector from robot to target
    double directionX = targetCoords.x - robotCoords.x;
    double directionY = targetCoords.y - robotCoords.y;

    // Normalize direction vector
    double length = sqrt(directionX * directionX + directionY * directionY);
    directionX /= length;
    directionY /= length;

    // Calculate perpendicular vector
    double perpendicularX = -directionY;
    double perpendicularY = directionX;

    // Scale perpendicular vector by distance
    perpendicularX *= (AVOID_DISTANCE + ROBOT_WIDTH);
    perpendicularY *= (AVOID_DISTANCE + ROBOT_WIDTH);

    // Calculate new point coordinates
    coords newPoint;
    newPoint.x = targetCoords.x + perpendicularX;
    newPoint.y = targetCoords.y + perpendicularY;

    return newPoint;
}


double calcDistToPath(coords *robotCoords, object *obs, double global_x, double global_y){
	// Convert two points into a line for the path
	float A = global_y - (robotCoords->y);
	float B = (robotCoords->x) - global_x;
	float C = (robotCoords->y) * (-B) - (A * (robotCoords->x));

	float top = abs((A * (robotCoords->x)) + (B * (robotCoords->y)) + C);
	float bot = sqrt((A*A) + (B*B));

	return top/bot;
}

double move_forward(oi_t *sensor_data, coords *robotCoords, double distance_mm) {
    double sum = 0;
    int power = 10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum < distance_mm + MOVEOFFSET) {
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
        else if (distance_mm - sum < power  && power > 20)
            power -= 15;

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

    while (sum > -distance_mm - MOVEOFFSET) {
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

double ram(oi_t *sensor, coords *robotCoords)
{
    double dist = 0;
    oi_setWheels(500 + TWISTOFFSET,500 - TWISTOFFSET);
    while(dist<800 && !sensor->bumpLeft && !sensor->bumpRight)
    {
        oi_update(sensor);
        dist += sensor->distance;
    }
    move_backward(sensor, robotCoords, 700);

    return 0.0;
}

void manuever(oi_t *sensor_data, coords *robotCoords, float distance_mm){
    float distance = 0;
    oi_update(sensor_data);

    while (distance < distance_mm){
        oi_update(sensor_data);
        distance += move_forward(sensor_data,robotCoords, distance);

        //cases for bumping either left or right
        if (sensor_data->bumpLeft && sensor_data->bumpRight){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords, 100);
            turn_right(sensor_data, robotCoords,90);
            distance += move_forward(sensor_data, robotCoords, 250);
        }

        //case for bumping left
        else if(sensor_data->bumpLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords,100);
            turn_right(sensor_data, robotCoords,90);
            distance += move_forward(sensor_data, robotCoords, 250);
            turn_left(sensor_data, robotCoords, 90);
        }

        //case for bumping right
        else if(sensor_data->bumpRight){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords,100);
            turn_left(sensor_data,robotCoords, 90);
            distance += move_forward(sensor_data, robotCoords, 250);
            turn_right(sensor_data,robotCoords, 90);
        }

        //case for hitting a cliff on the left
        else if(sensor_data->cliffLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords,50);
            turn_right(sensor_data, robotCoords,90);
            distance += move_forward(sensor_data, robotCoords,300);
            turn_left(sensor_data, robotCoords,90);
        }

        //case for hitting a cliff on the right
        else if(sensor_data->cliffRight){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords,50);
            turn_left(sensor_data,robotCoords, 90);
            distance += move_forward(sensor_data, robotCoords,300);
            turn_right(sensor_data, robotCoords,90);
        }

        //case for hitting a cliff on the front left
        else if(sensor_data->cliffFrontLeft){
            oi_update(sensor_data);
            move_backward(sensor_data, robotCoords,50);
            turn_right(sensor_data, robotCoords,90);
            distance += move_forward(sensor_data,robotCoords, 300);
        }

        //case for hitting a cliff on the front right
        else if(sensor_data->cliffFrontRight){
            oi_update(sensor_data);
            move_backward(sensor_data,robotCoords, 50);
            turn_left(sensor_data,robotCoords, 90);
            distance += move_forward(sensor_data,robotCoords, 300);
        }


        //case for hitting the boundary
        //else if(sensor_data->cliffFrontLeftSignal > 2600)
    }
}
