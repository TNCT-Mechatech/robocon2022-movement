#ifndef _WHEEL_HPP_
#define _WHEEL_HPP_

#include <math.h>
#include <MD.hpp>
#include <Encoder.hpp>
#include <PID.hpp>


class Wheel
{
public:
    Wheel(MD *md, Encoder *encoder, PID::ctrl_param_t *pid_gain, double radius);

    //  drive wheel
    void drive(double velocity, double dt);

    //  stop wheel
    void stop();
private:
    //  module
    MD *_md;
    Encoder *_encoder;
    PID _pid;

    //  param
    double _radius;
    
    //  vel
    PID::ctrl_variable_t _vel;
};

#endif