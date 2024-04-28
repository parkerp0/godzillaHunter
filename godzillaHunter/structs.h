#ifndef STRUCTS_H
#define STRUCTS_H

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

#endif
