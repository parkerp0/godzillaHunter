#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdlib.h>

typedef struct {
    float x;
    float y;
    float heading;
} coords;

typedef struct
{
    float x;
    float y;
    float linearWidth;
}object;

extern coords *robotCoords;
extern float prevY;
extern float prevX;

int addObject(object** currentList, int obsCount, float x,float y, float linearWidth);//reutrns the new number of obstacles

#endif
