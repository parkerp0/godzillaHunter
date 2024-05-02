/*
 * scan.c
 *
 *  Created on: Apr 16, 2024
 *      Author: cdoran & lcano
 */
#include "scan.h"

#define objMatchThresh 10 //threshold for deciding if objects are the same might be mm idk

int count = 0;



object* scan(){
            int i = 0;              //Constants and counters for for loops
            int j = 0;
            int k = 0;
            char toPutty[100];      //Used to make the PuTTy output a message
            char *toPutty_ptr = toPutty;
            int IRmeasurement;
            float pingDistance;

            object *obs = NULL;

                    i = 0;
                    j = 0;
                    k = 0;

                    int sensorAngle[90] = {0};        //sensorAngle will be set to angles where there is an object
                    float sensorDistance[90] = {0};   //sensorDistance will be set to the distance to the object

                    sprintf(toPutty, "Degrees\tDistance (cm)\n\r");     //Sets up the degree and distance display before the scanner reads degrees and distances
                    uart_sendStr(toPutty_ptr);

                    for (i = 0; i <= 180; i = i + 2) {                              //Has the cybot scan every 2 degrees from 0 to 180 degrees
                        servo_move(i);
                        pingDistance = ping_getDistance();
                        IRmeasurement = IR_read();
                        if (IRmeasurement > 700) {                                //The cybot will detect an object when the IR output is greater than 900
                            sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);     //Sends the object angle and distance at that angle to the PuTTy
                            while (toPutty[j] != '\0') {
                                uart_sendChar(toPutty[j]);
                                j++;
                            }
                            sensorAngle[k] = i;                                     //Angle and distance at that angle are stored in their variables at element k, and k will increase after storing an angle and distance
                            sensorDistance[k] = pingDistance;
                            k++;


                        }
                        j = 0;
                    }


                    int avgAngle[10] = {0};     //Will be set to the average angle of the object
                    float avgDist[10] = {0};    //Will be set to the average distance to the object
                    int width[10] = {0};        //Will be set to the angular width of the object
                    int l = 0;                  //Counter that increases for each object
                    float StartDist[10] = {0};  //Distance at the object's starting angle
                    float EndDist[10] = {0};    //Distance at the object's ending angle

                    for (i = 1; i <= k; i++) {
                        if (StartDist[l] == 0) {
                            StartDist[l] = sensorDistance[i];                       //StartDist will have the distance at the start angle
                        }
                        if (sensorAngle[i] - sensorAngle[i-1] <= 4  && i != k) {    //Compares current angle to the last angle to tell if it is the same object
                            if (sensorAngle[i] - sensorAngle[i-1] == 4) {           //The sensor sometimes skips an angle, and this will prevent it from saying it is two separate objects
                                j = j + 4;                                          //The j's are used for width and finding average
                                avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];       //For setting up averages; j increases by 4, so the angle/distance should be multiplied by 4
                                avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                                EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle

                            }
                            else {
                                j = j + 2;                                          //The j's are used for width and finding average
                                avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];       //For setting up averages; j increases by 2, so the angle/distance should be multiplied by 2
                                avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                                EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle
                            }
                        }
                        else {
                            if (j <= 2) {                           //The scanner sometimes reads an object when there isn't one, so this will prevent it from affecting the data
                                j = 0;                              //It sets j, the averages, and the start distance back to 0 since there isn't an object
                                avgAngle[l] = 0;
                                avgDist[l] = 0;
                                StartDist[l] = 0;
                            }
                            else {                                  //At the end of an object
                                width[l] = j;                       //Sets angular width to j
                                avgAngle[l] = avgAngle[l] / j;      //Finds the averages
                                avgDist[l] = avgDist[l] / j;
                                j = 0;                              //Resets j for the next object
                                l++;                                //l increases for counting number of objects and storing object data in the arrays
                            }
                        }
                    }
                    count = l;

//                    //initialize the smallest width, angle, and distance
//                    int SmallestWidth = 1000;
//                    int SmallestWidthAngle = 0;
//                    int SmallestWidthDistance = 0;
//
//                    //initialize the largest width, angle, and distance
//                    int LargestWidth = 0;
//                    int LargestWidthAngle = 0;
//                    int LargestWidthDistance = 0;


                    float LinearWidth[10] = {0};                                                                     //Stores object linear width
                    for (i = 0; i < l; i++) {
                        LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));  //Uses Law of Cosines to find the linear width of each object
                    }

                    j = 0;
                    sprintf(toPutty, "\n\rObject\tAngle\tDistance\tWidth\n\r");                                     //Sets up the PuTTy display before reading off the object angle, distance, and width
                    while (toPutty[j] != '\0') {
                        uart_sendChar(toPutty[j]);
                        j++;
                    }

                    for (i = 0; i < l; i++) {
                        sprintf(toPutty, "%d\t%d\t%f\t%f\n\r", i + 1, avgAngle[i], avgDist[i], LinearWidth[i]);     //Prints each object's average angle, average distance, and linear width
                        j = 0;
                        while (toPutty[j] != '\0') {
                            uart_sendChar(toPutty[j]);
                            j++;
                        }
                    }

                    obs = malloc(sizeof(object)*(l+1));
                    for(i = 0,j=0; j<l; i++ ,j++)
                    {


                        float a = IR_SERVO_OFFSET+(avgDist[i]*1000)+(0.5 * LinearWidth[i]);

                    	float adjDist = sqrt((a * a)  // a squared
                    	+ ((SERVO_CENTER_OFFSET)*(SERVO_CENTER_OFFSET))  // b squared
						- (2*a*(SERVO_CENTER_OFFSET)*cos((avgAngle[i]+90) * degreesToRadians))); // 2*a*b*cos(theta) where theta is just the servo angle

						float adjAngle = asin(a * sin((avgAngle[i]+90)*degreesToRadians)/adjDist);

                    	 // Adjust the angle to be aligned with the robot's heading system


                        obs[j].x = adjDist*sin(adjAngle - (robotCoords->heading * degreesToRadians)) + robotCoords->x; //add current x/y coord to it
                        obs[j].y = adjDist*cos(adjAngle - (robotCoords->heading * degreesToRadians)) + robotCoords->y;
                        obs[j].linearWidth = LinearWidth[i] * 1000; //standardize to mm

                        sprintf(toPutty, "\n\rLinear Width in mm: %f\n\r", obs[j].linearWidth );
                        uart_sendStr(toPutty);

                        if(obs[j].x < 0 || obs[j].x >FIELD_WIDTH || obs[j].y < 0 || obs[j].y > FIELD_LENGTH)
                        {
                            j--;
                            l--;
                        }

                        sprintf(toPutty, "\n\rAdjDist: %f A:%f B:%f AdjAngle:%f \tl: %d\n\r", adjDist,a,SERVO_CENTER_OFFSET,adjAngle * radianToDegrees, l);
                        uart_sendStr(toPutty);
                        j = 0;//?

                    }
                    obs[l].x = 0.0;
                    obs[l].y = 0.0;
                    obs[l].linearWidth = 0.0;


//                    for (i = 0; i < l; i++) {
//                        if (LinearWidth[i] < SmallestWidth) {   //Finds the object with the smallest linear width and finds their angle and distance
//                            SmallestWidth = LinearWidth[i];
//                            SmallestWidthAngle = avgAngle[i];
//                            SmallestWidthDistance = avgDist[i];
//                        }
//                    }
                
//                    for (i = 0; i < l; i++) {
//                        if (LinearWidth[i] > LargestWidth) {   //Finds the object with the largest linear width and finds their angle and distance
//                            LargestWidth = LinearWidth[i];
//                            LargestWidthAngle = avgAngle[i];
//                            LargestWidthDistance = avgDist[i];
//                        }
//                    }

                            //print smallest object info to Putty
//                            sprintf(toPutty, "Smallest object width: %d\tAngle: %d\tDistance: %d\n\r", SmallestWidth, SmallestWidthAngle, SmallestWidthDistance);
//                            j = 0;
//                            while (toPutty[j] != '\0') {
//                            uart_sendChar(toPutty[j]);
//                            j++;
//                            }
//
//                            //print largest object info to Putty
//                            sprintf(toPutty, "Largest object width: %d\tAngle: %d\tDistance: %d\n\r", LargestWidth, LargestWidthAngle, LargestWidthDistance);
//                            m = 0;
//                            while (toPutty[m] != '\0') {
//                                uart_sendChar(toPutty[m]);
//                                m++;
//                            }
                
                    return obs;
}

int scanAndRewrite(object **currentObs,int obsCount)
{
    int i;
    int flag;
    char message[90];


    object *obsTemp = scan();
    while(obsTemp->linearWidth!= 0.0)
    {
        flag = 1;//assume that the new object is new
        for(i = 0; i<obsCount; i++)
        {
            if(vectorDifMag(obsTemp,&(*currentObs)[i]) < objMatchThresh)
            {//rewritten might work
                (*currentObs)[i].linearWidth = ((*currentObs)[i].linearWidth + obsTemp->linearWidth)/2;
                flag = 0;//the object is found to be not new
                break;
            }

        }
        if(flag) {
            obsCount = addObject(currentObs,obsCount,obsTemp->x,obsTemp->y,obsTemp->linearWidth);

            //run the Godzilla check here!!!!

            if (detectGodzilla(((*currentObs)[obsCount - 1])) == (*currentObs)[obsCount - 1]){
              //work in progress
              //Do something in the program to go ram Godzilla
              //if detectGozilla is successful, it returns the object you gave it
            }
        }

    obsTemp++;
    }
    for(i = 0; i<obsCount; i++)
    {
        sprintf(message,"obs %d: x: %.2f y:%.2f Width:%.2f\n\r",i,(*currentObs)[i].x,(*currentObs)[i].y,(*currentObs)[i].linearWidth);
        uart_sendStr(message);
    }//prints all objects
    
    //sprintf(message,"Hunter location x:%lf y:%lf heading: %lf\n\r", coord->x,coord->y,coord->heading);
    //uart_sendStr(message);
    free(obsTemp);

    return obsCount;
}

float vectorDifMag(object *obs,object *obs2)
{
    float newX = obs->x - obs2->x;
    float newY = obs->y - obs2->y;
    return sqrt((newX*newX) + (newY*newY));
}

//Finds and returns the largest object based on linearWidth. 
//*CALL AFTER* scanAndRewrite. Will break if you call it before

object* findLargestObj(object **currentObs, int obsCount) {

    int i;
    int m;

    int indexLargest = 0; // Start assuming the first object is the largest.
    float maxWidth = (*currentObs)[0].linearWidth; // Initial max width set to the first object's width.

    char toPutty[100];           //Used to make the PuTTy output a message
    char *toPutty_ptr = toPutty; //Used to make the PuTTy output a message


    //find the largest object in the struct
    for (i = 1; i < obsCount; i++) {
        if ((*currentObs)[i].linearWidth > maxWidth) {
            maxWidth = (*currentObs)[i].linearWidth;
            indexLargest = i;
        }
    }

    //print the largest object in the struct to Putty
    sprintf(toPutty, "\n\rLargest Object: %f\tX Coordinate: %f\tY Coordinate: %f\n\r", (*currentObs)[indexLargest].linearWidth, (*currentObs)[indexLargest].x, (*currentObs)[indexLargest].y);
    m = 0;
    while (toPutty[m] != '\0') {
        uart_sendChar(toPutty[m]);
        m++;
    }

    return &((*currentObs)[indexLargest]); // Return a pointer to the largest object found.
}


//called to detectGodzilla
object* detectGodzilla(object **currentObs, int obsCount) {
    if (obsCount == 0) return NULL; // No objects to check

    char toPutty[100];           //Used to make the PuTTy output a message
    char *toPutty_ptr = toPutty; //Used to make the PuTTy output a message


    //Test if object is Godzilla
    if ((*currentObs)[obsCount].linearWidth >= tMinLW && (*currentObs)[obsCount].linearWidth <= tMaxLW) {
        sprintf(toPutty,"\n\rGodzilla Sighting at: \tX Coordinate: %f\tY Coordinate: %f\t Linear Width: %f\n\r", (*currentObs)[obsCount].x, (*currentObs)[obsCount].y, (*currentObs)[obsCount].linearWidth);
        uart_sendStr(toPutty);
        sprintf(toPutty,"Confirming...");
        uart_sendStr(toPutty);
    } else {//scenario where Godzilla was not within the threshold
        sprintf(toPutty,"\n\rFalse Alarm... Not Godzilla.\n\r");
        uart_sendStr(toPutty);
        return NULL;
    }
    if (confirmGodzilla()){
        return &((*currentObs)[obsCount]); // Return a pointer to the Godzilla object found.
    } else { //scenario where Godzilla was not confirmed
        sprintf(toPutty,"\n\rFalse Alarm... Not Godzilla.\n\r");
        uart_sendStr(toPutty);
        return NULL;
    }
}

//called to confirm Godzilla
bool confirmGodzilla(){
    int confirmations = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        object* obsArray = scan();
        int i = 0;
        while (obsArray[i].linearWidth != 0.0) { //linearWidth of 0.0 indicates the end of objects
            if (obsArray[i].linearWidth >= tMinLW && obsArray[i].linearWidth <= tMaxLW) {
                confirmations++;
                break; // Stop checking once we find a valid object
            }
            i++;
        }
        free(obsArray); // frees allocated memory after use
        if (confirmations >= 3) {
            return true;
        }
    }
    return false;
}



