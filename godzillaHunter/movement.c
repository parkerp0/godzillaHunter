#include "movement.h"

extern coords *robotCoords;


int move_to_point(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir){
	sprintf(toPutty, "Moving To point: X: %lf\t Y: %lf\n\r", global_x, global_y);
	uart_sendStr(toPutty);

	//*numObs = scanAndRewrite(obs,*numObs);//add scan block to find more thing in between rerouting

	if(checkPerpendicularPoint((*obs),*numObs,global_x,global_y)!=1)return -1.0;

    // Sort obstacles based on distances to the robot
    qsort((*obs), *numObs, sizeof(object), compareDistances);



    int status = checkObstacles(sensor_data, obs, numObs, numAttempts, global_x, global_y, dir);
    if(checkPerpendicularPoint((*obs),*numObs,global_x,global_y)!=1)return -1.0;

    if (status == -1) return -1;

	sprintf(toPutty, "MTP After obs check: X: %.2f\t Y: %.2f\n\r", global_x, global_y);
	uart_sendStr(toPutty);

    float deltaX = global_x - robotCoords->x;
    float deltaY = global_y - robotCoords->y;
    
//    float deltaHeading = (robotCoords->heading) - targetHeading;

    
    float distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    if(distance > RESCAN_DIST)
    {
        move_to_point(sensor_data, obs, numObs,numAttempts, robotCoords->x + (deltaX/distance * (distance - RESCAN_DIST)), robotCoords->y + (deltaY/distance * (distance - RESCAN_DIST)), 1);
        *numObs = scanAndRewrite(obs,*numObs);
        checkObstacles(sensor_data, obs, numObs,numAttempts,global_x,global_y,dir);

        deltaX = global_x - robotCoords->x;
        deltaY = global_y - robotCoords->y;
        distance = sqrt(deltaX*deltaX + deltaY*deltaY);
    }

    float targetHeading = fmod(360 + (atan2(deltaX, deltaY) * 180.0 / M_PI), 360); // lock in the degrees to be -360 to 360
    float prevHeading = robotCoords->heading;
    set_heading(sensor_data, targetHeading);

    timer_waitMillis(500);

    status = move_forward(sensor_data, obs, numObs, distance, dir,numAttempts);
    if(status == -2.0f)
    {//if move forward fails due to bump try to move there again b/c bump correction has already fixed perpendicular
        move_to_point(sensor_data, obs, numObs,numAttempts,global_x,global_y, 1);
    }

    sprintf(toPutty, "TURNING BACK AFTER DODGE: prevHeading: %f\n\r", prevHeading);
    uart_sendStr(toPutty);

    set_heading(sensor_data, prevHeading);

    return 0;
}

int checkObstacles(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir){
//    int numObs = sizeof(*obs) / sizeof(object);

//	sprintf(toPutty, "numObs: %d\n\r", *numObs);
//	uart_sendStr(toPutty);

    if ( (numAttempts >= 4))
    	return -1.0; // return positive 1 because it isn't really an error, and idk if it should be considered a normal exit condition

    int j;
    for (j = 0; j < *numObs; j++) {
		sprintf(toPutty, "Obs %d at point: X: %lf\t Y: %lf\n\r", j, (*obs)[j].x, (*obs)[j].y);
		uart_sendStr(toPutty);
    }

    // Loop through sorted obstacles
    int i;
    for (i = 0; i < *numObs; i++) {

//    	sprintf(toPutty, "Inside of checkObs outer loop: i = %d\n\r", i);
//    	uart_sendStr(toPutty);



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
                sprintf(toPutty, "Obs at point: X: %lf\t Y: %lf is intersecting line X:%f Y:f\n\r", (*obs)[i].x, (*obs)[i].y, global_x, global_y);
                uart_sendStr(toPutty);

                coords newTarget = calculatePerpendicularPoint((*obs),*numObs,(*obs)[i], dir);
                if (newTarget.x == -1 && newTarget.y == -1 && newTarget.heading == -1){
                    return -1;
                }
            	sprintf(toPutty, "New target: X: %lf\t Y: %lf\n\r", newTarget.x, newTarget.y);
            	uart_sendStr(toPutty);

                // Recursively avoid each object in the path.
                int status = move_to_point(sensor_data, obs, numObs, numAttempts + 1, newTarget.x, newTarget.y, dir);
                int tempObsC = *numObs;
                *numObs = scanAndRewrite(obs,*numObs);
                if(tempObsC != *numObs || status == 2)
                     i = 0;

                sprintf(toPutty, "CHECK OBS STATUS AFTER ATTEMPT %d: %d\n\r\n\r", numAttempts, status);
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

    if (global_x < START_X || global_y < START_Y ||
                global_x > FIELD_WIDTH || global_y > FIELD_LENGTH) // Here x is the shorter way and y is the longer direction
        {
            //sprintf(toPutty, "POINT OUTSIDE OF FIELD: X: %f\t Y: %f\n\r", global_x, global_y);
            //uart_sendStr(toPutty);
            return -1;//point is out of the field
        }

    for (j = 0; j < numObs; j++)
    {
        // Check if there is an obstacle too close to the target location
        distToObs = sqrt((obs[j].x - global_x)*(obs[j].x - global_x) + (obs[j].y - global_y) * (obs[j].y - global_y));//how close it is
        padDist = (0.5 * obs[j].linearWidth) + (ROBOT_WIDTH * 0.5) + AVOID_DISTANCE; //closest it can be
        if(distToObs < padDist){
            //sprintf(toPutty, "POINT IS INTERSECTING A OBJECT: X: %f\t Y: %f\n\r", global_x, global_y);
            //uart_sendStr(toPutty);
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
            uart_sendStr("COULD NOT CALCULATE PERPENDICULAR POINT\n\r");
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
	sprintf(toPutty, "Obs at point: X: %lf\t Y: %lf\n\r", obs->x, obs->y);
	uart_sendStr(toPutty);
	// Convert two points into a line for the path
	float A = global_y - (robotCoords->y);
	float B = (robotCoords->x) - global_x;
	float C = ((robotCoords->y) * (-B)) - (A * (robotCoords->x));

	float top = fabsf((A * (obs->x)) + (B * (obs->y)) + C);
	float bot = sqrt((A*A) + (B*B));


	// sprintf(toPutty, "A: %.2f\tB: %.2f\tC: %.2f\tTop: %.2f\tBot: %.2f\tFinal: %.2f\n\r", A, B, C, top, bot, top/bot);
	// uart_sendStr(toPutty);

	return top/bot;
}

float move_forward(oi_t *sensor_data, object **obs, int *numObs, float distance_mm, int dir,int depth) {
	float sum = 0;
    int power = 10;
    int status;

    oi_update(sensor_data);
    lcd_printf("%lf", sensor_data->distance);

    while (sum < distance_mm + MOVEOFFSET) {
        oi_update(sensor_data);
        sum += sensor_data->distance;
        float deltaDistance = sensor_data->distance;
        float deltaHeading = -sensor_data->angle;

        robotCoords->heading += deltaHeading;
        robotCoords->heading = fmod(360 + (robotCoords->heading), 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);

        status = cliff_detected(sensor_data, obs, numObs, dir,depth);
        if (status == -1) {
            return -1.0;
        }else if(status == -2)
        {
            return -2.0;
        }


        if (distance_mm - sum > power  && power < 200)
            power += 10;
        else if (distance_mm - sum < power  && power > 20)
            power -= 15;

        oi_setWheels(power-TWISTOFFSET, power + TWISTOFFSET);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);

    }


	sprintf(toPutty, "Robot X: %f\tRobot Y: %f\tRobot Heading: %f\tPower: %d\n\r", robotCoords->x, robotCoords->y, robotCoords->heading, power);
	uart_sendStr(toPutty);
    oi_setWheels(0,0);
    return sum;
}

float move_backward(oi_t *sensor_data, float distance_mm)
{
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
        robotCoords->heading = fmod(360+ (robotCoords->heading), 360); // lock in the degrees to be -360 to 360

        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);

        if (distance_mm + sum > -power && power > -200)
            power -= 10;
        else if (distance_mm + sum < -power  && power < -20)
            power += 15;

        oi_setWheels(power + TWISTOFFSET, power - TWISTOFFSET);//change this
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }
    oi_setWheels(0,0);
    return sum;
}

void set_heading(oi_t *sensor_data,float degrees)
{
    float deltaHeading = degrees - (robotCoords->heading);

    if (deltaHeading >= 0){
        if (deltaHeading > 180){
            turn_left(sensor_data, 360-deltaHeading);
        } else {
            turn_right(sensor_data, deltaHeading);
        }
    } else {
        if (deltaHeading < -180) {
            turn_right(sensor_data, 360+deltaHeading);
        } else {
            turn_left(sensor_data, -deltaHeading);
        }
    }
    
}

float turn_right(oi_t *sensor,  float degrees) {
	float sum = 0;
    oi_setWheels(-150, 150);
    while (sum > -degrees + TURNOFFSET) {//+ 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;
        float deltaDistance = sensor->distance;
        float deltaHeading = -sensor->angle;

//        robotCoords->heading += deltaHeading;
//        robotCoords->heading = fmod(360 + (robotCoords->heading), 360); // lock in the degrees to be -360 to 360
//
//        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
//        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }

    robotCoords->heading += degrees;
    robotCoords->heading = fmod(360+ (robotCoords->heading), 360);


    oi_setWheels(0,0);
    sprintf(toPutty, "AFTER TURN RIGHT: %f\nX: %f\nY: %f\nA: %f\tDegrees: %f\n\r", sum, robotCoords->x, robotCoords->y, robotCoords->heading, degrees);
    uart_sendStr(toPutty);
    return sum;
}

float turn_left(oi_t *sensor, float degrees) {
	float sum = 0;
    oi_setWheels(150, -150);
    while (sum < degrees - TURNOFFSET) {//- 8.5 for robot 10
        oi_update(sensor);
        sum += sensor->angle;

        float deltaDistance = sensor->distance;
        float deltaHeading = -sensor->angle;

//        robotCoords->heading += deltaHeading;
//        robotCoords->heading = fmod(360+ (robotCoords->heading), 360);
//        robotCoords->heading = fmod(robotCoords->heading, 360); // lock in the degrees to be -360 to 360

//        robotCoords->x += deltaDistance * sin(robotCoords->heading * DEGREES_TO_RADS);
//        robotCoords->y += deltaDistance * cos(robotCoords->heading * DEGREES_TO_RADS);
        lcd_printf("%lf\nX: %lf\nY: %lf\nA: %lf", sum, robotCoords->x, robotCoords->y, robotCoords->heading);
    }

    robotCoords->heading += degrees;
    robotCoords->heading = fmod(360+ (robotCoords->heading), 360);


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

    return dist;
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


//pretty sure this is the correct godzilla function but leaving the other until I ask
float move_to_godzilla(oi_t *sensor_data, object *obs, int *numObs, object *godzilla, int dir){

    coords target = get_target_for_godzilla(obs, godzilla, numObs);

    if (target.x == -1 && target.y == -1 && target.heading == -1){
        sprintf(toPutty, "WARNING! COULD NOT NAVIGATE TO GODZILLA! get_target_for_godzilla\n\r");
        uart_sendStr(toPutty);
    }

    if (move_to_point(sensor_data,&obs,numObs,0,target.x,target.y,dir) == -1){ // basically if status == -1
        sprintf(toPutty, "WARNING! COULD NOT NAVIGATE TO GODZILLA! move_to_point\n\r");
        uart_sendStr(toPutty);
    }

    // Rotate to face Godzilla
    float deltaX = (target.x) - (robotCoords->x);
    float deltaY = (target.y) - (robotCoords->y);
    float targetHeading = fmod(360 + (atan2(deltaX, deltaY) * 180.0 / M_PI), 360); // lock in the degrees to be -360 to 360
    float deltaHeading = fabs(robotCoords->heading - targetHeading);

    // Figure out which way to turn
    if (targetHeading < robotCoords->heading)
        turn_left(sensor_data, deltaHeading);
    else
        turn_right(sensor_data, deltaHeading);

    lcd_printf("TurnDIR: %d\nTarget: %f\nCurrent: %f\nDelta: %f", (targetHeading < robotCoords->heading) ? -1 : 1, targetHeading, robotCoords->heading, deltaHeading);
    sprintf(toPutty, "TurnDIR: %d\nTarget: %f\nCurrent: %f\nDelta: %f\n\r", (targetHeading < robotCoords->heading) ? -1 : 1, targetHeading, robotCoords->heading, deltaHeading);
    uart_sendStr(toPutty);

    timer_waitMillis(500);
    // Return the distance to Godzilla so that we can get confirmation and hit it.
    return sqrt(deltaX*deltaX + deltaY*deltaY);

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

//Helper method for detecting cliffs and or objects when navigating
int cliff_detected(oi_t *sensor_data, object **obs, int *numObs, int dir, int depth){


    if (sensor_data->cliffLeft){//cliff
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*sin((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*cos((-60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, CLIFF_OBJECT_SIZE);
    } else if (sensor_data->cliffFrontLeft){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*sin((-20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*cos((260 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, CLIFF_OBJECT_SIZE);
    } else if (sensor_data->cliffFrontRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*sin((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*cos((20 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, CLIFF_OBJECT_SIZE);
    } else if (sensor_data->cliffRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*sin((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + CLIFF_OBJECT_SIZE/2.0)*cos((60 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, CLIFF_OBJECT_SIZE);
    } else if (sensor_data->bumpLeft){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((-45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((-45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } else if (sensor_data->bumpRight){
        *numObs = addObject(obs, *numObs, (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*sin((45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->x,
                  (ROBOT_WIDTH/2.0 + BUMP_OBJECT_WIDTH/2.0)*cos((45 + robotCoords->heading)*DEGREES_TO_RADS) + robotCoords->y, BUMP_OBJECT_WIDTH);
    } //removed boundry sensor options because boundry violations should not add objects
    //changed size of cliff violations to allow for the move_to_point to reroute well


    // TODO finalize the avoidance algorithm  
    if(
            sensor_data->cliffRightSignal > 2600
        || sensor_data->cliffLeftSignal > 2600
        || sensor_data->cliffFrontLeftSignal > 2600
        || sensor_data->cliffFrontRightSignal > 2600
      )
    {
        lcd_printf("On border reseting coords");
        uart_sendStr("On border reseting coords");
        
        int closeSideX = (robotCoords->x - START_X < FIELD_WIDTH - robotCoords->x) ? robotCoords->x - START_X : FIELD_WIDTH - robotCoords->x;
        int closeSideY = (robotCoords->y - START_Y < FIELD_LENGTH - robotCoords->y) ? robotCoords->y - START_Y : FIELD_LENGTH - robotCoords->y;

        if(closeSideX < closeSideY)//resets the x or y to the closest border to make the backup routine easier
            robotCoords->x = (robotCoords->x - START_X < FIELD_WIDTH - robotCoords->x) ? START_X - BOUNDRY_OVERSHOOT: FIELD_WIDTH  + BOUNDRY_OVERSHOOT;
        else
            robotCoords->y = (robotCoords->y - START_Y < FIELD_LENGTH - robotCoords->y) ? START_Y - BOUNDRY_OVERSHOOT: FIELD_LENGTH + BOUNDRY_OVERSHOOT;

        move_backward(sensor_data,150);
        turn_right(sensor_data,90);
        return -1.0;//break out of current move to point and try to force the obscheck to run again
    }
    else if(
           sensor_data->bumpLeft
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

            return -1;
        }
        sprintf(toPutty, "New target after BUMP/CLIFF: X: %lf\t Y: %lf\n\r", newTarget.x, newTarget.y);
        uart_sendStr(toPutty);

        // Recursively avoid each object in the path.
        int status = move_to_point(sensor_data, obs, numObs, depth, newTarget.x, newTarget.y, dir);
        if (status == -1) return -1;

//        turn_right(sensor_data, 180.0);
//        manuever(sensor_data, 400.0);
        return -2;
    } else {
    	lcd_clear();
    }
    return 1;
}
