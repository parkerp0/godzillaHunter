#include "structs.h"

coords *robotCoords;

float prevY;
float prevX;

int addObject(object** currentList, int obsCount, float x,float y, float linearWidth)
{
    obsCount++;
    (*currentList) = realloc((*currentList), sizeof(object)*obsCount);
    (*currentList)[obsCount-1].x = x;
    (*currentList)[obsCount-1].y = y;
    (*currentList)[obsCount-1].linearWidth = linearWidth;

    return obsCount;
}
