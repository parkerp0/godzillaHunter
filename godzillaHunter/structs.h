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
    int age;
}object;

extern coords *robotCoords;
extern float prevY;
extern float prevX;
static int totalObj;

#define MAXOBJCOUNT 10

int addObject(object** currentList, int obsCount, float x,float y, float linearWidth);//returns the new number of obstacles

int trimObj(object **currentObs, int obsCount);//

int compareObjAge(const void *a, const void *b);

#endif
