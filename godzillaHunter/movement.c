#include "movement.h"

extern coords *robotCoords;

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
//            robotCoords->x = START_X;
//            robotCoords->y = START_Y;
//            robotCoords->heading = 0;
//
//            oi_setWheels(0,0);
//
//            int numObs = 1;
//            object *obs = malloc(sizeof(object) * numObs);
////
////            obs[0].x = 500 + START_X; // mm
////            obs[0].y = 500 + START_Y;
////            obs[0].linearWidth = 2.54*4; // about 4 inches wide (in mm)
////
////
////            obs[1].x = 200; // mm
////            obs[1].y = 600;
////            obs[1].linearWidth = 2.54*4; // about 4 inches wide (in mm)
////
////
////
////            obs[2].x = 300; // mm
////            obs[2].y = 1500;
////            obs[2].linearWidth = 2.54*4; // about 4 inches wide (in mm)
//
////            object godzilla;
////            godzilla.x = 2500;
////            godzilla.y = 2500;
////            godzilla.linearWidth = 20;
//
//
//            lcd_printf("Move To Point test");
//            oi_setWheels(0,0);
//        	uart_sendStr("-----------------------------Move to Point test--------------------------------\n\r");
////            int status = move_to_point(sensorD, obs, &numObs, 0, 1000+START_X, 1000+START_Y, 1);
////        	turn_right(sensorD, 45);
////        	    int status = 0;
////        	int status = move_to_godzilla(sensorD, obs, &numObs, &godzilla, 1);
//
//            timer_waitMillis(2000);
//        	turn_right(sensorD, 45);
//            timer_waitMillis(2000);
//            move_forward(sensorD, &obs, &numObs, 500, 1);
//            timer_waitMillis(2000);
//            turn_left(sensorD, 45);
//            timer_waitMillis(2000);
//            move_forward(sensorD, &obs, &numObs, 500, 1);
//            timer_waitMillis(2000);
//            turn_right(sensorD, 180);
//            timer_waitMillis(2000);
//            move_forward(sensorD, &obs, &numObs, 1000, 1);
//            timer_waitMillis(2000);
//            turn_right(sensorD, 90);
//            timer_waitMillis(2000);
//            move_forward(sensorD, &obs, &numObs, 500, 1);
//
////            sprintf(toPutty, "FINAL STATUS: %d\n\r", status);
////            uart_sendStr(toPutty);
//
//            oi_setWheels(0,0);
////            oi_free(sensorD);
//            while(1);
//}



float move_to_point(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir){
	sprintf(toPutty, "Moving To point: X: %lf\t Y: %lf\n\r", global_x, global_y);
	uart_sendStr(toPutty);

	//*numObs = scanAndRewrite(obs,*numObs);//add scan block to find more thing in between rerouting

	if(checkPerpendicularPoint((*obs),*numObs,global_x,global_y)!=1)return -1.0;

    // Sort obstacles based on distances to the robot
    qsort((*obs), *numObs, sizeof(object), compareDistances);



    int status = checkObstacles(sensor_data, obs, numObs, numAttempts, global_x, global_y, dir);
    if(checkPerpendicularPoint((*obs),*numObs,global_x,global_y)!=1)return -1.0;
    sprintf(toPutty, "MTP STATUS AFTER ATTEMPT %d: %d\n\r", numAttempts, status);
    uart_sendStr(toPutty);

    // if (numAttempts == 0 && status == -1 && dir == 1) { // if it is the top level call and it failed going to the right, try going to the left
    //     status = move_to_point(sensor_data, obs, numObs, numAttempts, global_x, global_y, -1);
    // }

    if (status == -1) return -1;

	sprintf(toPutty, "MTP After obstacles: X: %.2f\t Y: %.2f\n\r", global_x, global_y);
	uart_sendStr(toPutty);

    float deltaX = global_x - robotCoords->x;
    float deltaY = global_y - robotCoords->y;
    float targetHeading = fmod(atan2(deltaX, deltaY) * 180.0 / M_PI, 360); // lock in the degrees to be -360 to 360
    float deltaHeading = fabs((robotCoords->heading) - targetHeading);

    // Figure out which way to turn
    if (targetHeading < robotCoords->heading) // Turn left
        turn_left(sensor_data, deltaHeading);
    else
        turn_right(sensor_data, deltaHeading); // Turn right

    lcd_printf("TurnDIR: %d\nTarget: %f\nCurrent: %f\nDelta: %f", (targetHeading < robotCoords->heading) ? -1 : 1, targetHeading, robotCoords->heading, deltaHeading);


    timer_waitMillis(500);
    float distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    move_forward(sensor_data, obs, numObs, distance, dir);

    if (targetHeading < robotCoords->heading) // Turn left
        turn_right(sensor_data, deltaHeading);
    else
        turn_left(sensor_data, deltaHeading);

    return 0.0;
}

float checkObstacles(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir){
//    int numObs = sizeof(*obs) / sizeof(object);

//	sprintf(toPutty, "numObs: %d\n\r", *numObs);
//	uart_sendStr(toPutty);

    if ((numAttempts >= (*numObs)) || (numAttempts >= 4))
    	return -1.0; // return positive 1 because it isn't really an error, and idk if it should be considered a normal exit condition

    int j;
    for (j = numAttempts; j < *numObs; j++) {
		sprintf(toPutty, "Obs %d at point: X: %lf\t Y: %lf\n\r", j, (*obs)[j].x, (*obs)[j].y);
		uart_sendStr(toPutty);
    }

    // Loop through sorted obstacles
    int i;
    for (i = numAttempts; i < *numObs; i++) {

    	sprintf(toPutty, "Inside of checkObs outer loop: i = %d\n\r", i);
    	uart_sendStr(toPutty);

    	sprintf(toPutty, "Obs at point: X: %lf\t Y: %lf\n\r", (*obs)[i].x, (*obs)[i].y);
    	uart_sendStr(toPutty);

        // Calculate the vector from the robot to the target
    	float targetVectorX = global_x - robotCoords->x;
    	float targetVectorY = global_y - robotCoords->y;

        // Calculate the vector from the robot to the obstacle
    	float obstacleVectorX = (*obs)[i].x - robotCoords->x;
    	float obstacleVectorY = (*obs)[i].y - robotCoords->y;

        // Calculate the dot product of the two vectors
    	float dotProduct = targetVectorX * obstacleVectorX + targetVectorY * obstacleVectorY;

        // Check if the obstacle is between the robot and the target
        // Greater than 0 ensures that the obstacle is not behind or to the side of the robot.
        // the targetVector part ensures that the obstacle is not past the target vector.
        if (dotProduct > 0 && dotProduct < (targetVectorX * targetVectorX + targetVectorY * targetVectorY)) {


//        	uart_sendStr("Inside of the dot product check.\n\r");
//        	sprintf(toPutty, "Dot product: %f\tTargetVectorSquared: %f\n\r", dotProduct, (targetVectorX * targetVectorX + targetVectorY * targetVectorY));
//        	uart_sendStr(toPutty);
            // Obstacle lies between robot and target, calculate distPath
//            if (obs[i] == NULL) return 0.0;

            float distPath = calcDistToPath(&(*obs)[i], global_x, global_y);
            if ((distPath - ((*obs)[i].linearWidth / 2.0) - (ROBOT_WIDTH / 2.0)) <= AVOID_DISTANCE) {
                // Avoid obstacle
//            	sprintf(toPutty, "Dist to path: %f \n\r", distPath);
//            	uart_sendStr(toPutty);

//            	uart_sendStr("Inside of avoid obstacle part.\n\r");

                coords newTarget = calculatePerpendicularPoint((*obs),*numObs,(*obs)[i], dir);
                if (newTarget.x == -1 && newTarget.y == -1 && newTarget.heading == -1){
                    uart_sendStr("COULD NOT CALCULATE PERPENDICULAR POINT\n\r");
                    return -1;
                }
            	sprintf(toPutty, "New target: X: %lf\t Y: %lf\n\r", newTarget.x, newTarget.y);
            	uart_sendStr(toPutty);

                // Recursively avoid each object in the path.
                int status = move_to_point(sensor_data, obs, numObs, numAttempts + 1, newTarget.x, newTarget.y, dir);
                int tempObsC = *numObs;
                *numObs = scanAndRewrite(obs,*numObs);

                sprintf(toPutty, "oldObs : %d newObs: %d\n\r", tempObsC, *numObs);
                uart_sendStr(toPutty);
                if(tempObsC != *numObs)
                     i = 0;

                sprintf(toPutty, "CHECK OBS STATUS AFTER ATTEMPT %d: %d\n\r", numAttempts, status);
                uart_sendStr(toPutty);

                if (status == -1)
                    return -1;

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


int checkPerpendicularPoint(object *obs, int numObs, float global_x, float global_y){
    int j;
    float distToObs;
    float padDist;

    if (global_x < (ROBOT_WIDTH/2.0) || global_y < (ROBOT_WIDTH/2.0) ||
                global_x > (FIELD_WIDTH - (ROBOT_WIDTH/2.0)) || global_y > (FIELD_LENGTH - (ROBOT_WIDTH/2.0))) // Here x is the shorter way and y is the longer direction
        {
            sprintf(toPutty, "POINT OUTSIDE OF FIELD: X: %f\t Y: %f\n\r", global_x, global_y);
            uart_sendStr(toPutty);
            return -1;//point is out of the field
        }

    for (j = 0; j < numObs; j++)
    {
        // Check if there is an obstacle too close to the target location
        distToObs = sqrt((obs[j].x - global_x)*(obs[j].x - global_x) + (obs[j].y - global_y) * (obs[j].y - global_y));//how close it is
        padDist = (0.5 * obs[j].linearWidth) + (ROBOT_WIDTH * 0.5) + AVOID_DISTANCE; //closest it can be
        if(distToObs < padDist){
            sprintf(toPutty, "POINT IS INTERSECTING A OBJECT: X: %f\t Y: %f\n\r", global_x, global_y);
            uart_sendStr(toPutty);
            return 0;//point is inside of a robot (or obstacle?)
        }
    }
    return 1;
}

coords calculatePerpendicularPoint(object *obs, int obsCount, object targetCoords, int dir) {
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

    sprintf(toPutty, "Calculate Perpendicular Point: (%lf, %lf) \n\r", perpendicularX, perpendicularY);
    uart_sendStr(toPutty);

    while(checkPerpendicularPoint(obs,obsCount,targetCoords.x - (dir * perpendicularX),targetCoords.y - (dir * perpendicularY))!= 1)
    {
        dir*=-1;//switch the direction that it checking
        perpendicularX *= 1.2;//scale the point until it finds somewhere it can go
        perpendicularY *= 1.2;

        sprintf(toPutty, "Calculate Perpendicular Point: (%lf, %lf) \n\r", targetCoords.x - (dir * perpendicularX), targetCoords.y - (dir * perpendicularY));
        uart_sendStr(toPutty);

        if (fabs(perpendicularY) >= FIELD_LENGTH || fabs(perpendicularX) >= FIELD_WIDTH){
            coords newPoint;
            newPoint.x = -1;
            newPoint.y = -1;
            newPoint.heading = -1;

            return newPoint;
        }
    }

    // Calculate new point coordinates
    coords newPoint;
    newPoint.x = targetCoords.x - (dir * perpendicularX);
    newPoint.y = targetCoords.y - (dir * perpendicularY);

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

float move_forward(oi_t *sensor_data, object **obs, int *numObs, float distance_mm, int dir) {
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

        if (cliff_detected(sensor_data, obs, numObs, dir) == -1) {
            return -1.0;
        }


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

//        robotCoords->heading += deltaHeading;
//        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
//
//        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
//        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
//        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }


    robotCoords->heading += degrees;
    robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
    lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    oi_setWheels(0,0);
    sprintf(toPutty, "AFTER TURN RIGHT: %f\nX: %f\nY: %f\nA: %f\tDegrees: %f\n\r", sum, robotCoords->x, robotCoords->y, robotCoords->heading, degrees);
    uart_sendStr(toPutty);
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

//        robotCoords->heading += deltaHeading;
//        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
//
//        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
//        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
//        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }


    robotCoords->heading -= degrees;
    robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360
    lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    oi_setWheels(0,0);
    sprintf(toPutty, "AFTER TURN LEFT: %f\nX: %f\nY: %f\nA: %f\tDegrees: %f\n\r", sum, robotCoords->x, robotCoords->y, robotCoords->heading, degrees);
    uart_sendStr(toPutty);
    return sum;
}


float ram(oi_t *sensor_data)
{
	float dist = 0;
    oi_setWheels(500 + TWISTOFFSET,500 - TWISTOFFSET);
    while(dist<800 && !sensor_data->bumpLeft && !sensor_data->bumpRight)
    {
        oi_update(sensor_data);
        dist += sensor_data->distance;
    }
  
    move_backward(sensor_data, 700);

    return 0.0;
}

coords get_target_for_godzilla(object *obs, object *godzilla, int *numObs){
    int numAttempts = 0;

    float robotAngle = atan2f((godzilla->y) - (robotCoords->y), (godzilla->x) - (robotCoords->x));
    float angle = robotAngle + M_PI; // the angle of the line between the robot and godzilla

    sprintf(toPutty, "BEFORE WHILE LOOP! %lf\t%lf\t%lf\t%lf\t%lf\n\r",numAttempts * (AVOID_DISTANCE / (1.0 *GODZILLA_RAM_DISTANCE)), numAttempts*1.0, AVOID_DISTANCE, GODZILLA_RAM_DISTANCE, (robotAngle + (3 * M_PI))* (180.0/M_PI));
    uart_sendStr(toPutty);

    // Iterative version to save memory
    while ((angle) < (robotAngle + (3 * M_PI))){ // keep it from going in circles forever
        angle += (M_PI/12);

        sprintf(toPutty, "Attempt: %d\tAngle: %lf\n\r", numAttempts, angle * (180.0/M_PI));
        uart_sendStr(toPutty);

        // This should work I'm pretty sure
        coords newPoint;
        newPoint.x = (godzilla->x) + ((GODZILLA_RAM_DISTANCE) * cos(angle));
        newPoint.y = (godzilla->y) + ((GODZILLA_RAM_DISTANCE) * sin(angle));

        // TODO if we have time make it check both clockwise and ccw at the same time and choose whichever we get first so it is closer to the robot

        // Print it
        sprintf(toPutty, "GODZILLA TARGET LOCATION before obs check: (%lf, %lf)\n\r", newPoint.x, newPoint.y);
	    uart_sendStr(toPutty);

        // Check if there is an obstacle too close to the target point or along the path to godzilla
        int j;
        for (j = 0; j < *numObs; j++) {
            // Check if the obstacle is between the robot and the target
            // Greater than 0 ensures that the obstacle is not behind or to the side of the robot.
            // the targetVector part ensures that the obstacle is not past the target vector.
            // Check if there is an obstacle too close to the target location
            if ((((obs[j].linearWidth/2.0) + sqrt(((obs[j].x-(newPoint.x))*(obs[j].x-(newPoint.x))) +
                             ((obs[j].y-(newPoint.y))*(obs[j].y-(newPoint.y))))) <= ((ROBOT_WIDTH/2.0) + (AVOID_DISTANCE)))){
                numAttempts++;
                continue; // go on to the next possible location
            }

            // Calculate the vector from the target to godzilla
            float targetVectorX = godzilla->x - newPoint.x;
            float targetVectorY = godzilla->y - newPoint.y;

            // Calculate the vector from the obstacle to the target
            float obstacleVectorX = obs[j].x - newPoint.x;
            float obstacleVectorY = obs[j].y - newPoint.y;

            // Calculate the dot product of the two vectors
            float dotProduct = targetVectorX * obstacleVectorX + targetVectorY * obstacleVectorY;
            if (dotProduct > 0 && dotProduct < (targetVectorX * targetVectorX + targetVectorY * targetVectorY)) {

                if ((calcDistToPathGodzilla(&obs[j], godzilla, newPoint, numObs) - (obs[j].linearWidth / 2.0) - (ROBOT_WIDTH / 2.0)) <= AVOID_DISTANCE){
                    numAttempts++;
                    continue; // go on to the next possible location
                }
            }

            sprintf(toPutty, "GODZILLA TARGET LOCATION CONFIRMED: (%lf, %lf)\n\r", newPoint.x, newPoint.y);
            uart_sendStr(toPutty);
            return newPoint;
        }
    }

    coords newPoint;
    newPoint.x = -1;
    newPoint.y = -1;
    newPoint.heading = -1;

    return newPoint;
}

float calcDistToPathGodzilla(object *obs, object *godzilla, coords target, int *numObs){
	// Convert two points into a line for the path
	float A = (godzilla->y) - (robotCoords->y);
	float B = (robotCoords->x) - (godzilla->x);
	float C = ((robotCoords->y) * (-B)) - (A * (robotCoords->x));

	float top = fabsf((A * (obs->x)) + (B * (obs->y)) + C);
	float bot = sqrt((A*A) + (B*B));


	sprintf(toPutty, "GODZILLA A: %.2f\tB: %.2f\tC: %.2f\tTop: %.2f\tBot: %.2f\tFinal: %.2f\n\r", A, B, C, top, bot, top/bot);
	uart_sendStr(toPutty);

	return top/bot;
}

void manuever(oi_t *sensor_data, float distance_mm){
//    float distance = 0;
//    oi_update(sensor_data);
//
//    while (distance < distance_mm){
//        oi_update(sensor_data);
//        distance += move_forward(sensor_data, distance);
//
//        //cases for bumping either left and right
//        if (sensor_data->bumpLeft && sensor_data->bumpRight){
//            oi_update(sensor_data);
//            move_backward(sensor_data, 100);
//            turn_right(sensor_data,90);
//            distance += move_forward(sensor_data, 250);
//        }
//
//        //case for bumping left
//        else if(sensor_data->bumpLeft){
//            oi_update(sensor_data);
//            move_backward(sensor_data,100);
//            turn_right(sensor_data,90);
//            distance += move_forward(sensor_data, 250);
//            turn_left(sensor_data, 90);
//        }
//
//        //case for bumping right
//        else if(sensor_data->bumpRight){
//            oi_update(sensor_data);
//            move_backward(sensor_data,100);
//            turn_left(sensor_data, 90);
//            distance += move_forward(sensor_data, 250);
//            turn_right(sensor_data, 90);
//        }
//
//        //case for hitting a cliff on the left
//        else if(sensor_data->cliffLeft){
//            oi_update(sensor_data);
//            move_backward(sensor_data,50);
//            turn_right(sensor_data,90);
//            distance += move_forward(sensor_data,300);
//            turn_left(sensor_data,90);
//        }
//
//        //case for hitting a cliff on the right
//        else if(sensor_data->cliffRight){
//            oi_update(sensor_data);
//            move_backward(sensor_data,50);
//            turn_left(sensor_data, 90);
//            distance += move_forward(sensor_data,300);
//            turn_right(sensor_data,90);
//        }
//
//        //case for hitting a cliff on the front left
//        else if(sensor_data->cliffFrontLeft){
//            oi_update(sensor_data);
//            move_backward(sensor_data,50);
//            turn_right(sensor_data,90);
//            distance += move_forward(sensor_data, 300);
//        }
//
//        //case for hitting a cliff on the front right
//        else if(sensor_data->cliffFrontRight){
//            oi_update(sensor_data);
//            move_backward(sensor_data, 50);
//            turn_left(sensor_data, 90);
//            distance += move_forward(sensor_data, 300);
//        }
//
//
//        //case for hitting the boundary
//        //else if(sensor_data->cliffFrontLeftSignal > 2600)
//    }
}

//Helper method for detecting cliffs and or objects when navigating
int cliff_detected(oi_t *sensor_data, object **obs, int *numObs, int dir){


    if (sensor_data->cliffLeft){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffLeftSignal > 2600){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffFrontLeft){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((260 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffFrontRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffFrontLeftSignal > 2600){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((260 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffFrontRightSignal > 2600){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->cliffRightSignal > 2600){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->bumpLeft){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((-45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->bumpRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } 


    // TODO finalize the avoidance algorithm  
    if(sensor_data->cliffRightSignal > 2600
                    || sensor_data->cliffLeftSignal > 2600
                    || sensor_data->cliffFrontLeftSignal > 2600
                    || sensor_data->cliffFrontRightSignal > 2600
                    || sensor_data->bumpLeft
                    || sensor_data->bumpRight
                    || sensor_data->cliffFrontLeft
                    || sensor_data->cliffFrontRight
                    || sensor_data->cliffRight
                    || sensor_data->cliffLeft)
    {
        // Turns around and maneuvers away
        lcd_printf("CLIFF DETECTED!!!!");
        uart_sendStr("CLIFF OR BUMP DETECTED !!!");

        // print all objects
        int i;
        for(i = 0; i< (*numObs); i++)
        {
            sprintf(toPutty,"obs %d: x: %.2f y:%.2f Width:%.2f\n\r",i,(*obs)[i].x,(*obs)[i].y,(*obs)[i].linearWidth);
            uart_sendStr(toPutty);
        }

        move_backward(sensor_data, 150); // move back slightly

        // Move perpendicular to it to avoid the object and continue the path
        coords newTarget = calculatePerpendicularPoint((*obs),*numObs,(*obs)[(*numObs)-1], dir);
        if (newTarget.x == -1 && newTarget.y == -1 && newTarget.heading == -1){
            uart_sendStr("COULD NOT CALCULATE PERPENDICULAR POINT\n\r");
            return -1;
        }
        sprintf(toPutty, "New target after BUMP/CLIFF: X: %lf\t Y: %lf\n\r", newTarget.x, newTarget.y);
        uart_sendStr(toPutty);

        // Recursively avoid each object in the path.
        int status = move_to_point(sensor_data, obs, numObs, 0, newTarget.x, newTarget.y, dir);
        if (status == -1) return -1;

//        turn_right(sensor_data, 180.0);
//        manuever(sensor_data, 400.0);
        return 0;
    } else {
    	lcd_clear();
    }
    return 0;
}
