#include <Encoder.hpp>
#include <chrono>
#include <cstdio>

Encoder::Encoder(PinName channel_a, PinName channel_b, int revolution)
    :_A(channel_a,PullDown), _B(channel_b,PullDown)
{
    //  init parameters
    _count = 0;
    _last_angle = 0;
    //  revolution
    _revolution = revolution * 1;
    
    //  set up attach
    _A.rise(callback(this, &Encoder::_AR));
    // _A.fall(callback(this, &Encoder::_AF));
    // _B.rise(callback(this, &Encoder::_BR));
    // _B.fall(callback(this, &Encoder::_BF));
}

double Encoder::get_revolution()
{
    return (double)_count / (double)_revolution;
}

double Encoder::get_rps(double dt)
{
    if(dt)
    {
        //  RPS
        _angle_velocity = (get_revolution() - _last_angle) / dt;
        //  update
        _last_angle = get_revolution();
        
        return _angle_velocity;
    }
    return 0;
}

int Encoder::get_count(){
    return _count;
}

void Encoder::reset(){
    _last_angle = 0;
    _count = 0;
}


//  Encoder Interrupt
void Encoder::_AR(){
    if (_B.read()) {
        _count--;
    } else if (!_B.read()) {
        _count++;
    }
}
void Encoder::_AF(){
    if (_B.read()) {
        _count++;
    } else if (!_B.read()) {
        _count--;
    }
}
void Encoder::_BR(){
    if (_A.read()) {
        _count++;
    } else if (!_A.read()) {
        _count--;
    }
}
void Encoder::_BF(){
    if (_A.read()) {
        _count--;
    } else if (!_A.read()) {
        _count++;
    }
}