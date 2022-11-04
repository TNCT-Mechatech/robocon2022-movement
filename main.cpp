#include "AnalogIn.h"
#include "mbed.h"

/////////////////////////////
//  Private include

//  Serial Bridge
#include <SerialBridge.hpp>
#include <MbedHardwareSerial.hpp>
#include <MessageStructure.hpp>
#include <Controller.hpp>
#include <ShooterMessage.hpp>
#include <MovementFeeback.hpp>

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
#include <cstdio>

/////////////////////////////
//  Private definition

//  LOOP RATE
#define LOOP_RATE 30

//  RX Timeout
#define RX_TIMEOUT 1000

//  LEDs
#define RX_LED PB_15
#define RX_TIMEOUT_LED PB_1

//  MD
#define MD_MAX_DUTY 1.0
#define MD_1_PWM D5
#define MD_1_DIR D6
#define MD_2_PWM D4
#define MD_2_DIR D7
#define MD_3_PWM D3
#define MD_3_DIR D8
#define MD_4_PWM D2
#define MD_4_DIR D9

//  Encoder
#define ENCODER_REVOLUTION 4800
#define ENCODER_DIFF_MS 20
#define ENCODER_1_A PA_14
#define ENCODER_1_B PA_13
#define ENCODER_2_A PC_10
#define ENCODER_2_B PC_12
#define ENCODER_3_A PB_9
#define ENCODER_3_B PB_8
#define ENCODER_4_A PC_3
#define ENCODER_4_B PC_2

//  Wheel 半径 m (127mm)
#define WHEEL_RADIUS 0.0635

//  Omuni4 中心からの距離 m (494.19mm)
#define OMUNI_4_RADIUS 0.49419


//  Serial Bridge
#define UART_SLAVE_TX PA_11
#define UART_SLAVE_RX PA_12
//  Message ID
#define CONTROLLER_RX_ID 0
#define FEEDBACK_TX_ID 1
#define CONTROLLER_TX_ID 10

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
// SerialDev *pc_dev = new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
// SerialBridge pc_serial(pc_dev, 1024);
// SerialDev *slave_dev = new MbedHardwareSerial(new BufferedSerial(UART_SLAVE_TX, UART_SLAVE_RX, 115200));
// SerialBridge slave_serial(slave_dev, 1024);
//  Serial Bridge Message
Controller controller_msg;
ShooterMessage shooter_msg;
MovementFeedback movement_feedback_msg;


//  DEBUG
AnalogIn analog(A0);
double power = 0;
double prev_power = 0;

/////////////////////////////
//  Private protype function

//  initializer for md,encoder,pid,omuni, etc.
static void initialize_module();


int main()
{
    //  initialize module
    initialize_module();

    //  start timers
    rx_timer.start();

    //  Serial Bridge
    //  to pc
    // pc_serial.add_frame(CONTROLLER_RX_ID, &controller_msg);
    // pc_serial.add_frame(FEEDBACK_TX_ID, &movement_feedback_msg);
    //  to slave
    // slave_serial.add_frame(CONTROLLER_TX_ID, &shooter_msg);

    while (true) {

        /* 
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
        if(pc_serial.update() == 0)
        {
            //  succeed communication
            
            if(controller_msg.was_updated())
            {   
                //  send another board
                shooter_msg.data.all_reload = controller_msg.data.all_reload;
                shooter_msg.data.shooter.num = controller_msg.data.shooter.num;
                shooter_msg.data.shooter.power = controller_msg.data.shooter.power;
                shooter_msg.data.shooter.action = controller_msg.data.shooter.action;


                //  set variable
                drive_variable.x = controller_msg.data.movement.x;
                drive_variable.y = controller_msg.data.movement.y;
                drive_variable.z = controller_msg.data.movement.z;

                pc_serial.write(CONTROLLER_TX_ID);

                //  Toggle LED
                rx_led = !rx_led;
                rx_timeout_led = false;
                //  reset timer
                rx_timer.reset();
            }
        }

        */

        
        
        //  feedback
        wheel[0]->get_state(
            &(movement_feedback_msg.data.target.v1),
            &(movement_feedback_msg.data.output.v1)
        );
        wheel[1]->get_state(
            &(movement_feedback_msg.data.target.v2),
            &(movement_feedback_msg.data.output.v2)
        );
        wheel[2]->get_state(
            &(movement_feedback_msg.data.target.v3),
            &(movement_feedback_msg.data.output.v3)
        );
        wheel[3]->get_state(
            &(movement_feedback_msg.data.target.v4),
            &(movement_feedback_msg.data.output.v4)
        );

        /*
        //  send
        pc_serial.write(FEEDBACK_TX_ID);
        
        //  Drive omuni4
        omuni4->drive(
            drive_variable.x, 
            drive_variable.y,
            drive_variable.z
        );
        */

        
        power = ((analog.read() - 0.5) * 2) * 0.6 + prev_power * 0.4;
        prev_power = power;
        //  Drive omuni4
        omuni4->drive(
            0, 
            0,
            power * M_PI * 0.5
        );

        printf(
            "t:%.3f a:%.3f b:%.3f c:%.3f d:%.3f\n\r",
            movement_feedback_msg.data.target.v1,
            movement_feedback_msg.data.output.v1,
            movement_feedback_msg.data.output.v2,
            movement_feedback_msg.data.output.v3,
            movement_feedback_msg.data.output.v4
        );

        // printf("%d\n\r", encoder[2]->get_count());

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
        MD_MAX_DUTY,
        false
    );
    md[1] = new MD(
        MD_2_PWM,
        MD_2_DIR,
        MD_MAX_DUTY,
        true
    );
    md[2] = new MD(
        MD_3_PWM,
        MD_3_DIR,
        MD_MAX_DUTY,
        true
    );
    md[3] = new MD(
        MD_4_PWM,
        MD_4_DIR,
        MD_MAX_DUTY,
        true
    );

    //  init Encoder
    encoder[0] = new Encoder(
        ENCODER_1_A,
        ENCODER_1_B,
        ENCODER_REVOLUTION
    );
    encoder[1] = new Encoder(
        ENCODER_2_A,
        ENCODER_2_B,
        ENCODER_REVOLUTION
    );
    encoder[2] = new Encoder(
        ENCODER_3_A,
        ENCODER_3_B,
        ENCODER_REVOLUTION
    );
    encoder[3] = new Encoder(
        ENCODER_4_A,
        ENCODER_4_B,
        ENCODER_REVOLUTION
    );

    //  pid gain
    pid_param[0] = PID::ctrl_param_t {
        0.0,
        0.0,
        0.0,
        1 / 4.0,
        false
    };
    pid_param[1] = PID::ctrl_param_t {
        0.0,
        0.0,
        0.0,
        1 / 4.0,
        false
    };
    pid_param[2] = PID::ctrl_param_t {
        0.0,
        0.0,
        0.0,
        1 / 4.0,
        false
    };
    pid_param[3] = PID::ctrl_param_t {
        0.0,
        0.0,
        0.0,
        1 / 4.0,
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
