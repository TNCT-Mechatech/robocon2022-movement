#include <Wheel.hpp>

Wheel::Wheel(MD *md, Encoder *encoder, PID::ctrl_param_t *pid_gain, double radius)
    : _md(md), _encoder(encoder), _pid(PID(&_vel, pid_gain))
{
    _vel = PID::ctrl_variable_t {0, 0, 0};
    _radius = radius;
}


void Wheel::drive(double velocity, double dt)
{
    //  convert m/s to RPS
    _vel.target = velocity / (2 * _radius * M_PI);
    //  feed back
    _vel.feedback = _encoder->get_rps(dt);

    //  calculate
    _pid.step(dt);

    //  drive
    _md->drive(_vel.output);
}

void Wheel::stop()
{
    _vel.target = 0.0;

    //  stop md
    _md->drive(0.0);
}

void Wheel::get_state(float *target, float *feedback)
{
    *target = _vel.target;
    *feedback = _vel.feedback;
}