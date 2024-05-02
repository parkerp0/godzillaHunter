#include "structs.h"

coords *robotCoords;

float prevY;
float prevX;

int totalObject = 0;

int addObject(object** currentList, int obsCount, float x,float y, float linearWidth)
{
    obsCount++;
    (*currentList) = realloc((*currentList), sizeof(object)*obsCount);
    (*currentList)[obsCount-1].x = x;
    (*currentList)[obsCount-1].y = y;
    (*currentList)[obsCount-1].linearWidth = linearWidth;
    (*currentList)[obsCount-1].age = totalObject;
    totalObject++;

    if(totalObject > MAXOBJCOUNT)obsCount = trimObj(currentList,obsCount);

    return obsCount;
}

int trimObj(object **currentObs, int obsCount)
{
    qsort((*currentObs), obsCount, sizeof(object), compareObjAge);
    obsCount--;
    (*currentObs) = realloc((*currentObs),sizeof(object) * obsCount);
    object temp;

    temp.x = (*currentObs)->x;
    temp.y = (*currentObs)->y;
    temp.linearWidth = (*currentObs)->linearWidth;
    temp.age = (*currentObs)->age;

    (*currentObs)->x = (*currentObs)[obsCount-1].x;
    (*currentObs)->y = (*currentObs)[obsCount-1].y;
    (*currentObs)->linearWidth = (*currentObs)[obsCount-1].linearWidth;
    (*currentObs)->age = (*currentObs)[obsCount-1].age;

    (*currentObs)[obsCount-1].x = temp.x;
    (*currentObs)[obsCount-1].y = temp.y;
    (*currentObs)[obsCount-1].linearWidth = temp.linearWidth;
    (*currentObs)[obsCount-1].age = temp.age;

    return obsCount;

}

int compareObjAge(const void *a, const void *b)
{
    object* aObs = (object*)a;
    object* bObs = (object*)b;
    return bObs->age - aObs->age;
}
