#ifndef OBJECT_H
#define OBJECT_H

struct Object
{
    float m;     /* Mass of the object */
    float k;     /* Damping constant */
    float F_max; /* Max force applied to the object */
    float F_min; /* Min force applied to the object */
    float T;     /* Time step */
    float v;     /* Velocity of the object */
    float z;     /* Position of the object */
};

float Object_Step(struct Object *obj, float F);

#endif
