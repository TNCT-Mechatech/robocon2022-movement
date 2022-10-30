#ifndef _MD_HPP_
#define _MD_HPP_

#include "mbed.h"
#include "math.h"

class MD
{
public:
    MD(PinName pwm, PinName dir, double max_duty = 1.0, bool default_dir = false);
    
    /**
     * Drive md
     * @params power
     */
    void drive(double power);
    
private:
    //  GPIO
    PwmOut _pwm;
    DigitalOut _dir;

    //  duty limit
    double _max_duty;
    
    //  default dir
    bool _default_dir;
};
#endif