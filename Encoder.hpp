#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include <mbed.h>

#define M_PI 3.14159265358979323846

class Encoder
{
public:
    Encoder(PinName channel_a, PinName channel_b, int revolution, float time);
    
    void reset();
    
    double get_angle();
    double get_angle_velocity();
    
    int get_count();
    
private:
    //  Encoder Pin
    InterruptIn _A, _B;
    //  revolution
    int _revolution;
    
    //  count
    int _count;
    int _old_count;
    double _angle_velocity;
    
    //  interrupt
    void _AR();
    void _AF();
    void _BR();
    void _BF();
    //  update parameters ticker
    Ticker _ticker;
    float _time;
    void _sa();
};

#endif