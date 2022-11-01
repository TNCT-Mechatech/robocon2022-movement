#ifndef _OMUNI_4_HPP_
#define _OMUNI_4_HPP_

#include <Wheel.hpp>
#include <math.h>

#define WHEEL_1_RAD (double)3 / (double)4 * M_PI
#define WHEEL_2_RAD (double)5 / (double)4 * M_PI
#define WHEEL_3_RAD (double)7 / (double)4 * M_PI
#define WHEEL_4_RAD (double)1 / (double)4 * M_PI
#define SQRT2 sqrt(2)

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