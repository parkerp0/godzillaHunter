/*
 * scan.c
 *
 *  Created on: Apr 16, 2024
 *      Author: cdoran & lcano
 */
#include "Timer.h"
#include "scan.h"
#include "open_interface.h"
#include "lcd.h"
#include "uart-interrupt.h"
#include "servo.h"
#include "ping.h"

#define objMatchThresh 0.03 //threshold for deciding if objects are the same

object* scan()
{
    int i = 0;              //Constants and counters for for loops
    int j = 0;
    int k = 0;
    int m = 0;
    char toPutty[100];      //Used to make the PuTTy output a message
    char *toPutty_ptr = toPutty;
    int IRmeasurement;
    float pingDistance;
    
    object *obs = NULL;
    i = 0;
    j = 0;
    k = 0;
    m = 0;
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
    obs[l].x = 0.0;
    obs[l].y = 0.0;
    obs[l].linearWidth = 0.0;//makes the last object 0 to avoid passing another var
    for(i = 0; i<l; i++)
    {
        obs[i].x = avgDist[i]*sin(avgAngle[i]*degreesToRadians);//add current x/y coord to it
        obs[i].y = avgDist[i]*cos(avgAngle[i]*degreesToRadians);
        obs[i].linearWidth = LinearWidth[i];
    }
        
    return obs;
}

object* scanAndRewrite(object *currentObs,int obsCount)
{
    obsTemp = scan();
    while(obsTemp->linearWidth!= 0.0)
    {
        for(i = 0; i<obsCount; i++)
        {
            if(vectorDifMag(obsTemp,&currentObs[i]) < objMatchThresh)
            {//rewritten might work
                currentObs[i].linearWidth = (currentObs[i].linearWidth + obsTemp->linearWidth)/2;
            }else
                flag = 1;
        }
        if(flag)
        {
            obsCount++;
            currentObs = realloc(obs,sizeof(object)*obsCount);
            //obsCopy(obs[obsCount-1],obsTemp);
            currentObs[obsCount-1].x = obsTemp->x;
            currentObs[obsCount-1].y = obsTemp->y;
            currentObs[obsCount-1].linearWidth = obsTemp->linearWidth;
            flag = 0;
        }
        
    obsTemp++;
    }
    for(i = 0; i<obsCount; i++)
    {
        sprintf(message,"obs %d: x: %.2f y:%.2f Width:%.2f\n\r",i,obs[i].x,obs[i].y,obs[i].linearWidth);
        uart_sendStr(message);
    }//prints all objects
    
    sprintf(message,"Hunter location x:%lf y:%lf heading: %lf\n\r", coord->x,coord->y,coord->heading);
    uart_sendStr(message);
    free(obsTemp);
}

float vectorDifMag(object *obs,object *obs2)
{
    int newX = obs->x - obs2->x;
    int newY = obs->y - obs2->y;
    return sqrt((newX*newX) + (newY*newY));
}