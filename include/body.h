#ifndef BODY_H_INCLUDED
#define BODY_H_INCLUDED

#include "proj_headers.h"

class body{
    Vector2 pos;
    Vector2 speed;
    Vector2 accel;
public:

}

class circumferece : body{
    double r;
}

class triangle : body{
    double s;
}

class rectangle : body {
    double h,w;
}




#endif // BODY_H_INCLUDED
