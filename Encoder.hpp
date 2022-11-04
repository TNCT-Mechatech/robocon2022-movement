#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include "DigitalOut.h"
#include <mbed.h>

#define M_PI 3.14159265358979323846

class Encoder
{
public:
    Encoder(PinName channel_a, PinName channel_b, int revolution);
    
    void reset();
    
    double get_revolution();
    double get_rps(double dt);
    
    int get_count();
    
private:
    //  Encoder Pin
    InterruptIn _A, _B;
    // InterruptIn _A;
    // DigitalIn _B;
    //  revolution
    int _revolution;
    
    //  count
    int _count;
    double _last_angle;
    double _angle_velocity;
    
    //  interrupt
    void _AR();
    void _AF();
    void _BR();
    void _BF();
};

#endif