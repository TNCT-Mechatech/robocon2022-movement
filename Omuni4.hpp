#ifndef _OMUNI_4_HPP_
#define _OMUNI_4_HPP_

#include <Wheel.hpp>
#include <math.h>

#define SQRT2 1.41421356237

class Omuni4
{
public:
    Omuni4(Wheel *wheel[4], double radius);

    void drive(double x, double y, double theta);

    void stop();
private:
    //  timer for pid
    Timer timer;
    double _previous_time;

    //  distance of wheel and center of robots
    double _radius;

    Wheel *_wheel[4];
};
#endif