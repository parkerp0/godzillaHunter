#include "movement.h"

extern coords *robotCoords;

//coords *robotCoords = NULL;

//int main (void) {
//
//            oi_t *sensorD = oi_alloc();
//            oi_init(sensorD);
//
//            timer_init();
//            lcd_init();
//            uart_interrupt_init();
//            button_init();
//
//
//
//            robotCoords = malloc(sizeof(coords));
//            robotCoords->x = 0;
//            robotCoords->y = 0;
//            robotCoords->heading = 0;
//
//            oi_setWheels(0,0);
//
//            int numObs = 3;
//            object *obs = malloc(sizeof(object) * numObs);
//
//            obs[0].x = -550; // mm
//            obs[0].y = -550;
//            obs[0].linearWidth = 2.54*4; // about 4 inches wide (in mm)
//
//
//            obs[1].x = 200; // mm
//            obs[1].y = 600;
//            obs[1].linearWidth = 2.54*4; // about 4 inches wide (in mm)
//
//
//
//            obs[2].x = 300; // mm
//            obs[2].y = 1500;
//            obs[2].linearWidth = 2.54*4; // about 4 inches wide (in mm)
//
//
//            lcd_printf("Move To Point test");
//        	uart_sendStr("-----------------------------Move to Point test--------------------------------\n\r");
//            move_to_point(sensorD, obs, numObs, 0, 1000, 1000);
//
//            oi_setWheels(0,0);
////            oi_free(sensorD);
//            while(1);
//}



float move_to_point(oi_t *sensor_data, object *obs, int numObs, int numAttempts, float global_x, float global_y){
	sprintf(toPutty, "Moving To point: X: %lf\t Y: %lf\n\r", global_x, global_y);
	uart_sendStr(toPutty);

	// TODO check if the point is outside of the field.

    // Sort obstacles based on distances to the robot
    qsort(obs, numObs, sizeof(object), compareDistances);

    checkObstacles(sensor_data, obs, numObs, numAttempts, global_x, global_y);
	sprintf(toPutty, "MTP After obstacles: X: %.2f\t Y: %.2f\n\r", global_x, global_y);
	uart_sendStr(toPutty);

    float deltaX = global_x - robotCoords->x;
    float deltaY = global_y - robotCoords->y;
    float targetHeading = fmod(atan2(deltaX, deltaY) * 180.0 / M_PI, 360); // lock in the degrees to be -360 to 360
    float deltaHeading = fabs(robotCoords->heading - targetHeading);

    // Figure out which way to turn
    int turnDir = 1; // default is turn right
    if (targetHeading < robotCoords->heading) // Turn left instead
        turnDir = -1;
	if (turnDir == 1)
		turn_right(sensor_data, deltaHeading);
	else
		turn_left(sensor_data, deltaHeading);

    lcd_printf("TurnDIR: %d\nTarget: %f\nCurrent: %f\nDelta: %f", turnDir, targetHeading, robotCoords->heading, deltaHeading);


    timer_waitMillis(500);
    float distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    move_forward(sensor_data, distance);


    return 0.0;
}

float checkObstacles(oi_t *sensor_data, object *obs, int numObs, int numAttempts, float global_x, float global_y){
//    int numObs = sizeof(*obs) / sizeof(object);

	sprintf(toPutty, "numObs: %d\n\r", numObs);
	uart_sendStr(toPutty);

    // TODO check if the target point is inside of an obstacle zone


    if (numAttempts >= numObs)
    	return 0.0;

    int j;
    for (j = numAttempts; j < numObs; j++) {
		sprintf(toPutty, "Obs %d at point: X: %lf\t Y: %lf\n\r", j, obs[j].x, obs[j].y);
		uart_sendStr(toPutty);
    }

    // Loop through sorted obstacles
    int i;
    for (i = numAttempts; i < numObs; i++) {

    	sprintf(toPutty, "Inside of checkObs outer loop: i = %d\n\r", i);
    	uart_sendStr(toPutty);

    	sprintf(toPutty, "Obs at point: X: %lf\t Y: %lf\n\r", obs[i].x, obs[i].y);
    	uart_sendStr(toPutty);

        // Calculate the vector from the robot to the target
    	float targetVectorX = global_x - robotCoords->x;
    	float targetVectorY = global_y - robotCoords->y;

        // Calculate the vector from the robot to the obstacle
    	float obstacleVectorX = obs[i].x - robotCoords->x;
    	float obstacleVectorY = obs[i].y - robotCoords->y;

        // Calculate the dot product of the two vectors
    	float dotProduct = targetVectorX * obstacleVectorX + targetVectorY * obstacleVectorY;

        // Check if the obstacle is between the robot and the target
        // Greater than 0 ensures that the obstacle is not behind or to the side of the robot.
        // the targetVector part ensures that the obstacle is not past the target vector.
        if (dotProduct > 0 && dotProduct < (targetVectorX * targetVectorX + targetVectorY * targetVectorY)) {


        	uart_sendStr("Inside of the dot product check.\n\r");
//        	sprintf(toPutty, "Dot product: %f\tTargetVectorSquared: %f\n\r", dotProduct, (targetVectorX * targetVectorX + targetVectorY * targetVectorY));
//        	uart_sendStr(toPutty);
            // Obstacle lies between robot and target, calculate distPath
//            if (obs[i] == NULL) return 0.0;

            float distPath = calcDistToPath(&obs[i], global_x, global_y);
            if ((distPath - (obs[i].linearWidth / 2.0) - (ROBOT_WIDTH / 2.0)) <= AVOID_DISTANCE) {
                // Avoid obstacle
            	sprintf(toPutty, "Dist to path: %f \n\r", distPath);
            	uart_sendStr(toPutty);

            	uart_sendStr("Inside of avoid obstacle part.\n\r");

                coords newTarget = calculatePerpendicularPoint(obs[i]);
            	sprintf(toPutty, "New target: X: %lf\t Y: %lf\n\r", newTarget.x, newTarget.y);
            	uart_sendStr(toPutty);

                // Recursively avoid each object in the path.
                move_to_point(sensor_data, obs, numObs, numAttempts + 1, newTarget.x, newTarget.y);
                

            }
        }
    }

    return 0.0;
}


int compareDistances(const void *a, const void *b) {
    object *obsA = (object *)a;
    object *obsB = (object *)b;
    // Calculate distances to the robot for both objects
    double distA = calcDistToRobot(obsA);
    double distB = calcDistToRobot(obsB);
    // Compare distances
    if (distA < distB) return -1;
    else if (distA > distB) return 1;
    else return 0;
}

float calcDistToRobot(object *obs){
    if (obs == NULL) return 1; // make null values be last
	return sqrt(((obs->x-robotCoords->x)*(obs->x-robotCoords->x)) + ((obs->y-robotCoords->y)*(obs->y-robotCoords->y)));
}

coords calculatePerpendicularPoint(object targetCoords) {
    // Calculate direction vector from robot to target
	float directionX = targetCoords.x - robotCoords->x;
	float directionY = targetCoords.y - robotCoords->y;

    // Normalize direction vector
	float length = sqrt(directionX * directionX + directionY * directionY);
    directionX /= length;
    directionY /= length;

    // Calculate perpendicular vector
    float perpendicularX = -directionY;
    float perpendicularY = directionX;

    // Scale perpendicular vector by distance
    perpendicularX *= (AVOID_DISTANCE + ROBOT_WIDTH + targetCoords.linearWidth);
    perpendicularY *= (AVOID_DISTANCE + ROBOT_WIDTH + targetCoords.linearWidth);

    // Calculate new point coordinates
    coords newPoint;
    newPoint.x = targetCoords.x + perpendicularX;
    newPoint.y = targetCoords.y + perpendicularY;

    return newPoint;
}


float calcDistToPath(object *obs, float global_x, float global_y){


	sprintf(toPutty, "obs: %d   \n\r", sizeof(*obs));
	uart_sendStr(toPutty);
	sprintf(toPutty, "Obs at point: X: %lf\t Y: %lf\n\r", obs->x, obs->y);
	uart_sendStr(toPutty);
	// Convert two points into a line for the path
	float A = global_y - (robotCoords->y);
	float B = (robotCoords->x) - global_x;
	float C = ((robotCoords->y) * (-B)) - (A * (robotCoords->x));

	float top = fabsf((A * (obs->x)) + (B * (obs->y)) + C);
	float bot = sqrt((A*A) + (B*B));


	sprintf(toPutty, "A: %.2f\tB: %.2f\tC: %.2f\tTop: %.2f\tBot: %.2f\tFinal: %.2f\n\r", A, B, C, top, bot, top/bot);
	uart_sendStr(toPutty);

	return top/bot;
}

float move_forward(oi_t *sensor_data, float distance_mm) {
	float sum = 0;
    int power = 10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum < distance_mm + MOVEOFFSET) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        float deltaDistance = sensor_data->distance;
        float deltaHeading = -sensor_data->angle;

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

        //		sprintf(toPutty, "Robot X: %f\tRobot Y: %f\tRobot Heading: %f\tPower: %d\n\r", robotCoords->x, robotCoords->y, robotCoords->heading, power);
        //		uart_sendStr(toPutty);
    }


	sprintf(toPutty, "Robot X: %f\tRobot Y: %f\tRobot Heading: %f\tPower: %d\n\r", robotCoords->x, robotCoords->y, robotCoords->heading, power);
	uart_sendStr(toPutty);
    oi_setWheels(0,0);
    return sum;
}

float move_backward(oi_t *sensor_data, float distance_mm) {
	float sum = 0;
    int power = -10;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum > -distance_mm - MOVEOFFSET) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        float deltaDistance = sensor_data->distance;
        float deltaHeading = -sensor_data->angle;

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

float turn_right(oi_t *sensor,  float degrees) {
	float sum = 0;
    oi_setWheels(-200, 200);
    while (sum > -degrees + TURNOFFSET) {//+ 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        float deltaDistance = sensor->distance;
        float deltaHeading = -sensor->angle;

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

float turn_left(oi_t *sensor, float degrees) {
	float sum = 0;
    oi_setWheels(200, -200);
    while (sum < degrees - TURNOFFSET) {//- 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        float deltaDistance = sensor->distance;
        float deltaHeading = -sensor->angle;

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

float moveCalibrate(oi_t *sensor_data)
{
    lcd_printf("PRESS 4 To move 1 meter");
    while(button_getButton() != 4);
    move_forward(sensor_data, 1000);

    lcd_printf("PRESS 4 To move 2 meters");
    while(button_getButton() != 4);
    move_forward(sensor_data, 2000);
    return 0.0;
}

float turnCalibrate(oi_t *sensor_data)
{

    lcd_printf("PRESS 4 to start turn calibration");
    while(button_getButton() != 4);

    float sum;
    sum = turn_left(sensor_data, 90);
    while(1)
        {
            switch(button_getButton())
            {
            case(1):
                sum+=turn_left(sensor_data, 1);
                break;
            case(2):
                sum+=turn_right(sensor_data, 1);
                break;
            case(3):
                sum+=turn_left(sensor_data, 20);
                break;
            case(4):
                sum+=turn_right(sensor_data, 20);
                break;
            }
            lcd_printf("1: left 1 \n2: right 1 \n3:left 20 \n4:right 20\n%lf", sum);
        }
    return 0.0;
}

float ram(oi_t *sensor)
{
	float dist = 0;
    oi_setWheels(500 + TWISTOFFSET,500 - TWISTOFFSET);
    while(dist<800 && !sensor->bumpLeft && !sensor->bumpRight)
    {
        oi_update(sensor);
        dist += sensor->distance;
    }
  
    move_backward(sensor, 700);

    return 0.0;
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
            turn_right(sensor_data,90);
            distance += move_forward(sensor_data, 250);
        }

        //case for bumping left
        else if(sensor_data->bumpLeft){
            oi_update(sensor_data);
            move_backward(sensor_data,100);
            turn_right(sensor_data,90);
            distance += move_forward(sensor_data, 250);
            turn_left(sensor_data, 90);
        }

        //case for bumping right
        else if(sensor_data->bumpRight){
            oi_update(sensor_data);
            move_backward(sensor_data,100);
            turn_left(sensor_data, 90);
            distance += move_forward(sensor_data, 250);
            turn_right(sensor_data, 90);
        }

        //case for hitting a cliff on the left
        else if(sensor_data->cliffLeft){
            oi_update(sensor_data);
            move_backward(sensor_data,50);
            turn_right(sensor_data,90);
            distance += move_forward(sensor_data,300);
            turn_left(sensor_data,90);
        }

        //case for hitting a cliff on the right
        else if(sensor_data->cliffRight){
            oi_update(sensor_data);
            move_backward(sensor_data,50);
            turn_left(sensor_data, 90);
            distance += move_forward(sensor_data,300);
            turn_right(sensor_data,90);
        }

        //case for hitting a cliff on the front left
        else if(sensor_data->cliffFrontLeft){
            oi_update(sensor_data);
            move_backward(sensor_data,50);
            turn_right(sensor_data,90);
            distance += move_forward(sensor_data, 300);
        }

        //case for hitting a cliff on the front right
        else if(sensor_data->cliffFrontRight){
            oi_update(sensor_data);
            move_backward(sensor_data, 50);
            turn_left(sensor_data, 90);
            distance += move_forward(sensor_data, 300);
        }


        //case for hitting the boundary
        //else if(sensor_data->cliffFrontLeftSignal > 2600)
    }
}
//Helper method for detecting cliffs and or objects when navigating
bool cliff_detected(oi_t *sensor_data){
    if(sensor_data->cliffRight || sensor_data->cliffLeft  || sensor_data->bumpLeft ||
            sensor_data->bumpRight || sensor_data->cliffFrontLeft || sensor_data->cliffFrontRight)
    {
        // Turns around and maneuvers away
        turn_right(sensor_data, 180.0);
        manuever(sensor_data, 400.0);
        return true;
    }
    return false;
}
