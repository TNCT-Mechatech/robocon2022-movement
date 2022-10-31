#include <Encoder.hpp>
#include <chrono>

Encoder::Encoder(PinName channel_a, PinName channel_b, int revolution, int time_ms)
    :_A(channel_a,PullDown),_B(channel_b,PullDown)
{
    //  init parameters
    _count = 0;
    _old_count = 0;
    //  revolution
    _revolution = revolution * 4;

    //  set up ticker
    std::chrono::milliseconds millisec(time_ms);
    _time_sec = std::chrono::duration_cast<std::chrono::seconds>(millisec).count();

    if(time_ms != 0){
        _ticker.attach(callback(this,&Encoder::_sa), std::chrono::duration_cast<std::chrono::microseconds>(millisec));
    }
    
    //  set up attach
    _A.rise(callback(this, &Encoder::_AR));
    _A.fall(callback(this, &Encoder::_AF));
    _B.rise(callback(this, &Encoder::_BR));
    _B.fall(callback(this, &Encoder::_BF));
}

double Encoder::get_revolution()
{
    return (double)_count / (double)_revolution;
}

double Encoder::get_rps()
{
    return _angle_velocity;    
}

int Encoder::get_count(){
    return _count;
}

void Encoder::reset(){
    _old_count = 0;
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

//  Update parameters
void Encoder::_sa(){
    //  RPS
    _angle_velocity = (((double)_count - (double)_old_count) / (double)_revolution) / _time_sec;
    
    //  update
    _old_count = _count;
}