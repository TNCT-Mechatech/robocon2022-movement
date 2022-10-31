#include "mbed.h"

/////////////////////////////
//  Private include

//  Serial Bridge
#include <SerialBridge.hpp>
#include <MbedHardwareSerial.hpp>
#include <Controller.hpp>

//  MD
#include <MD.hpp>
//  Encoder
#include <Encoder.hpp>
//  PID
#include <PID.hpp>
//  Wheel
#include <Wheel.hpp>
//  Omuni4
#include <Omuni4.hpp>

/////////////////////////////
//  Private definition

//  LOOP RATE
#define LOOP_RATE 30

//  RX Timeout
#define RX_TIMEOUT 1000

//  LEDs
#define RX_LED A0
#define RX_TIMEOUT_LED A0

//  MD
#define MD_MAX_DUTY 1.0
#define MD_1_PWM A0
#define MD_1_DIR A0
#define MD_2_PWM A0
#define MD_2_DIR A0
#define MD_3_PWM A0
#define MD_3_DIR A0
#define MD_4_PWM A0
#define MD_4_DIR A0

//  Encoder
#define ENCODER_REVOLUTION 10000
#define ENCODER_DIFF_MS 20
#define ENCODER_1_A A0
#define ENCODER_1_B A0
#define ENCODER_2_A A0
#define ENCODER_2_B A0
#define ENCODER_3_A A0
#define ENCODER_3_B A0
#define ENCODER_4_A A0
#define ENCODER_4_B A0

//  Wheel 半径 m (127mm)
#define WHEEL_RADIUS 0.0635

//  Omuni4 中心からの距離 m (494.19mm)
#define OMUNI_4_RADIUS 0.49419


//  Serial Bridge
#define CONTROLLER_RX_ID 0
#define CONTROLLER_TX_ID 1

/////////////////////////////
//  Private variable

//  LEDs
DigitalOut rx_led(RX_LED);
DigitalOut rx_timeout_led(RX_TIMEOUT_LED);

//  Modules
MD *md[4];
Encoder *encoder[4];
PID::ctrl_param_t pid_param[4];
Wheel *wheel[4];
Omuni4 *omuni4;

//  Drive variable
vector3_t drive_variable = {0, 0, 0};


//  Timers
//  RX timeout timer
Timer rx_timer;

//  Serial Bridge
SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
SerialBridge serial(dev, 1024);
//  Serial Bridge Message
Controller controller_msg[2];


/////////////////////////////
//  Private protype function

//  initializer for md,encoder,pid,omuni, etc.
static void initialize_module();


int main()
{
    //  start timers
    rx_timer.start();

    //  Serial Bridge
    serial.add_frame(CONTROLLER_RX_ID, &controller_msg[0]);
    serial.add_frame(CONTROLLER_TX_ID, &controller_msg[1]);

    while (true) {
        //  rx timeout
        if(std::chrono::duration_cast<std::chrono::milliseconds>(rx_timer.elapsed_time()).count() > RX_TIMEOUT)
        {
            // Toggle RX_TIMEOUT LED
            rx_timeout_led = true;

            //  set zero all control value
            drive_variable.x = 0;
            drive_variable.y = 0;
            drive_variable.z = 0;
        }

        //  Serial Bridge
        if(serial.update() == 0)
        {
            //  succeed communication
            
            if(controller_msg[0].was_updated())
            {   
                //  send another board
                controller_msg[1].data.movement.x = controller_msg[0].data.movement.x;
                controller_msg[1].data.movement.y = controller_msg[0].data.movement.y;
                controller_msg[1].data.movement.z = controller_msg[0].data.movement.z;

                //  set variable
                drive_variable.x = controller_msg[0].data.movement.x;
                drive_variable.y = controller_msg[0].data.movement.y;
                drive_variable.z = controller_msg[0].data.movement.z;

                serial.write(CONTROLLER_TX_ID);

                //  Toggle LED
                rx_led = !rx_led;
                rx_timeout_led = false;
                //  reset timer
                rx_timer.reset();
            }
        }

        //  Drive omuni4
        omuni4->drive(
            drive_variable.x, 
            drive_variable.y,
            drive_variable.z
        );

        // ThisThread::sleep_for(LOOP_RATE);
        thread_sleep_for(LOOP_RATE);
    }
}



static void initialize_module()
{
    //  init MD
    md[0] = new MD(
        MD_1_PWM,
        MD_1_DIR,
        MD_MAX_DUTY
    );
    md[1] = new MD(
        MD_2_PWM,
        MD_2_DIR,
        MD_MAX_DUTY
    );
    md[2] = new MD(
        MD_3_PWM,
        MD_3_DIR,
        MD_MAX_DUTY
    );
    md[3] = new MD(
        MD_4_PWM,
        MD_4_DIR,
        MD_MAX_DUTY
    );

    //  init Encoder
    encoder[0] = new Encoder(
        ENCODER_1_A,
        ENCODER_1_B,
        ENCODER_REVOLUTION,
        ENCODER_DIFF_MS
    );
    encoder[1] = new Encoder(
        ENCODER_2_A,
        ENCODER_2_B,
        ENCODER_REVOLUTION,
        ENCODER_DIFF_MS
    );
    encoder[2] = new Encoder(
        ENCODER_3_A,
        ENCODER_3_B,
        ENCODER_REVOLUTION,
        ENCODER_DIFF_MS
    );
    encoder[3] = new Encoder(
        ENCODER_4_A,
        ENCODER_4_B,
        ENCODER_REVOLUTION,
        ENCODER_DIFF_MS
    );

    //  pid gain
    pid_param[0] = PID::ctrl_param_t {
        0.1,
        0.0,
        0.0,
        0.0,
        false
    };
    pid_param[1] = PID::ctrl_param_t {
        0.1,
        0.0,
        0.0,
        0.0,
        false
    };
    pid_param[2] = PID::ctrl_param_t {
        0.1,
        0.0,
        0.0,
        0.0,
        false
    };
    pid_param[3] = PID::ctrl_param_t {
        0.1,
        0.0,
        0.0,
        0.0,
        false
    };

    //  wheel
    wheel[0] = new Wheel(
        md[0],
        encoder[0],
        &pid_param[0],
        WHEEL_RADIUS
    );
    wheel[1] = new Wheel(
        md[1],
        encoder[1],
        &pid_param[1],
        WHEEL_RADIUS
    );
    wheel[2] = new Wheel(
        md[2],
        encoder[2],
        &pid_param[2],
        WHEEL_RADIUS
    );
    wheel[3] = new Wheel(
        md[3],
        encoder[3],
        &pid_param[3],
        WHEEL_RADIUS
    );

    //  Omuni4
    omuni4 = new Omuni4(
        wheel,
        OMUNI_4_RADIUS
    );
}
