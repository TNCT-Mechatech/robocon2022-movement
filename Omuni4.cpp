#include <Omuni4.hpp>
#include <chrono>

/**
 *  @brief Omuni wheel movement
 *
 *  W2----W1
 *  | .  . |
 *  |  .   |
 *  | .  . |
 *  W3----W4
 */
Omuni4::Omuni4(Wheel *wheel[4], double radius)
{
    //  distance of wheel and center of robots
    _radius = radius;

    //  set wheels
    for (int i = 0; i < 4; i++) {
        _wheel[i] = wheel[i];
    }

    //  start timer
    timer.start();
}


/**
 * @brief drive Omuni4
 *
 * @param x vector of x (m/s) 
 * @param y vector of y (m/s)
 * @param theta vector of rotation (rad/s)
 */
void Omuni4::drive(double x, double y, double theta)
{
    double present_sec = chrono::duration<float>{timer.elapsed_time()}.count();
    double diff_sec = present_sec - _previous_time;
    //  w1
    _wheel[0]->drive(
        -1 * x * SQRT2 + y * SQRT2 + theta * _radius,
        diff_sec
    );
    //  w2
    _wheel[1]->drive(
        -1 * x * SQRT2 +  -1 * y * SQRT2 + theta * _radius,
        diff_sec
    );
    //  w3
    _wheel[2]->drive(
        x * SQRT2 + -1 * y * SQRT2 + theta * _radius,
        diff_sec
    );
    //  w4
    _wheel[3]->drive(
        x * SQRT2 + y * SQRT2 + theta * _radius,
        diff_sec
    );

    //  set time
    _previous_time = present_sec;
}

void Omuni4::stop()
{
    for (int i = 0; i < 4; i++) {
        _wheel[i]->stop();
    }
}