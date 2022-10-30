#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <Message.hpp>

typedef struct Vector3Type
{
    float x;
    float y;
    float z;
} vector3_t;

typedef struct ControllerType
{
    //  include vector-x,y,theta
    vector3_t movement;
} controller_t;

//  create message
typedef sb::Message<controller_t> Controller;

#endif