#include "mbed.h"

/////////////////////////////
//  Private include

//  Mbed
#include "DigitalOut.h"

//  Serial Bridge
#include <SerialBridge.hpp>
#include <MbedHardwareSerial.hpp>
#include <MessageStructure.hpp>
#include <Controller.hpp>
#include <ShooterMessage.hpp>
#include <MovementFeeback.hpp>
#include <DebugMessage.h>
#include <Gesture.h>
#include <PIDGain.hpp>
#include <Emotion.hpp>

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
#include <cstdint>
#include <cstdio>

//  std
#include <math.h>

/////////////////////////////
//  Private definition

//  LOOP RATE
#define LOOP_RATE 30

//  RX Timeout
#define RX_TIMEOUT 1000
#define SUB_CONTROLLER_TIMEOUT 1000

//  LEDs
#define CONTROLLER_RX_LED PB_2
// #define RX_TIMEOUT_LED PB_1
#define RX_TIMEOUT_LED PB_15
#define GESTURE_RX_LED PB_14
#define SUB_CONTROLLER_LED PB_13

//  Emergency STOP
#define EMERGENCY_STOP PC_8

//  MD
#define MD_MAX_DUTY 1.0
#define MD_1_DIR PC_4
#define MD_1_PWM D6
#define MD_2_DIR PB_12
#define MD_2_PWM D7
#define MD_3_DIR PC_5
#define MD_3_PWM D8
#define MD_4_DIR PC_6
#define MD_4_PWM D9

//  Encoder
#define ENCODER_REVOLUTION 1296
#define ENCODER_DIFF_MS 20
#define ENCODER_1_A PA_14
#define ENCODER_1_B PA_13
#define ENCODER_2_A PC_12
#define ENCODER_2_B PC_10
#define ENCODER_3_A PB_8
#define ENCODER_3_B PB_9
#define ENCODER_4_A PC_3
#define ENCODER_4_B PC_2

//  Wheel 半径 m (127mm)
#define WHEEL_RADIUS 0.0635

//  Omuni4 中心からの距離 m (494.19mm)
#define OMUNI_4_RADIUS 0.49419

//  speed (m/s, rad/s)
#define MAX_XY_SPEED 2.5
#define MAX_THETA_SPEED 1.57079632679
#define MOVEMENT_FAST_XY 1.0
#define MOVEMENT_FAST_THETA 0.7
#define MOVEMENT_NORMAL_XY 0.3
#define MOVEMENT_NORMAL_THETA 0.4
#define MOVEMENT_SLOW_XY 0.1
#define MOVEMENT_SLOW_THETA 0.1

//  smooth acceralation (m/sec/loop)
#define MOVEMENT_ACCERALATION 0.05
#define MOVEMENT_FIT_RANGE 0.10

//  Serial Bridge
#define UART_SLAVE_TX PA_11
#define UART_SLAVE_RX PA_12

//  IM920
#define IM920_TX PA_15
#define IM920_RX PB_7


//  Message ID
//  from pc
#define CONTROLLER_RX_ID 0
#define GESTURE_RX_ID 1
#define PID_GAIN_ID 2
#define FEEDBACK_TX_ID 5
#define DEBUG_TX_ID 6
#define UPSTREAM_CONTROLLER_ID 7
//  slave
#define CONTROLLER_TX_ID 10
#define EMOTION_ID 11
//  from sub controlle
#define SUB_CONTROLLER_ID 0

/////////////////////////////
//  Private variable

//  LEDs
DigitalOut rx_led(CONTROLLER_RX_LED);
DigitalOut rx_timeout_led(RX_TIMEOUT_LED);
DigitalOut rx_gesture_led(GESTURE_RX_LED);
DigitalOut rx_sub_ctl_led(SUB_CONTROLLER_LED);

//  Emergency stop
DigitalOut emergency_stop(EMERGENCY_STOP);

//  Modules
MD *md[4];
Encoder *encoder[4];
PID::ctrl_param_t pid_param[4];
Wheel *wheel[4];
Omuni4 *omuni4;

//  Drive variable
vector3_t received_movement_variable = {0, 0, 0};
vector3_t drive_variable = {0, 0, 0};
vector3_t present_drive_variable = {0, 0, 0};
int8_t movement_mode = 0;

//  Timers
//  RX timeout timer
Timer rx_timer;
//  Timer for timestamp
Timer rx_timestamp;
int last_sub_ctl_response = 0;


//  Serial Bridge
SerialDev *pc_dev = new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
SerialBridge pc_serial(pc_dev, 1024);
SerialDev *slave_dev = new MbedHardwareSerial(new BufferedSerial(UART_SLAVE_TX, UART_SLAVE_RX, 115200));
SerialBridge slave_serial(slave_dev);
SerialDev *sub_ctl_dev = new MbedHardwareSerial(new BufferedSerial(IM920_TX, IM920_RX, 115200));
SerialBridge sub_ctl_serial(sub_ctl_dev);
//  Serial Bridge Message
Controller controller_msg;
Controller upstream_controller_msg;
Controller sub_controller_msg;
ShooterMessage shooter_msg;
MovementFeedback movement_feedback_msg;
DebugMessage debug_msg;
Gesture gesture_msg;
PIDGain pid_gain_msg;
Emotion emotion_msg;

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
    rx_timestamp.start();

    //  Serial Bridge
    //  to pc
    pc_serial.add_frame(CONTROLLER_RX_ID, &controller_msg);
    pc_serial.add_frame(GESTURE_RX_ID, &gesture_msg);
    pc_serial.add_frame(FEEDBACK_TX_ID, &movement_feedback_msg);
    pc_serial.add_frame(DEBUG_TX_ID, &debug_msg);
    pc_serial.add_frame(PID_GAIN_ID, &pid_gain_msg);
    pc_serial.add_frame(UPSTREAM_CONTROLLER_ID, &upstream_controller_msg);
    //  to slave
    slave_serial.add_frame(CONTROLLER_TX_ID, &shooter_msg);
    slave_serial.add_frame(EMOTION_ID, &emotion_msg);
    //  from sub controller
    sub_ctl_serial.add_frame(SUB_CONTROLLER_ID, &sub_controller_msg);

    while (true) {        
        //  rx timeout
        if(rx_timer.read_ms() > RX_TIMEOUT)
        {
            // Toggle RX_TIMEOUT LED
            rx_timeout_led = true;

            //  set zero all control value
            received_movement_variable.x = 0;
            received_movement_variable.y = 0;
            received_movement_variable.z = 0;

            //  emergency stop
            emergency_stop = 0;
        }

        //  Serial Bridge
        if(pc_serial.update() == 0)
        {
            //  succeed communication
            //  Controller
            if(controller_msg.was_updated())
            {
                if(rx_timestamp.read_ms() - last_sub_ctl_response > SUB_CONTROLLER_TIMEOUT)
                {
                    //  emergency stop
                    emergency_stop = controller_msg.data.emergency_switch;

                    //  send another board
                    shooter_msg.data.all_reload = controller_msg.data.all_reload;
                    shooter_msg.data.shooter.num = controller_msg.data.shooter.num;
                    shooter_msg.data.shooter.power = controller_msg.data.shooter.power;
                    shooter_msg.data.shooter.action = controller_msg.data.shooter.action;

                    emotion_msg.data.face = controller_msg.data.face;


                    //  set variable
                    received_movement_variable.x = controller_msg.data.movement.x;
                    received_movement_variable.y = controller_msg.data.movement.y;
                    received_movement_variable.z = controller_msg.data.movement.z;
                    movement_mode = controller_msg.data.movement_mode;

                    slave_serial.write(CONTROLLER_TX_ID);
                    slave_serial.write(EMOTION_ID);

                    rx_timeout_led = false;
                    //  reset timer
                    rx_timer.reset();
                }

                //  Toggle LED
                rx_led = !rx_led;
            }

            //  Gesture
            if(gesture_msg.was_updated())
            {
                shooter_msg.data.shooter.action = gesture_msg.data.type == 1 ? 3 : 0;

                //  send to slave
                slave_serial.write(CONTROLLER_TX_ID);

                rx_gesture_led = !rx_gesture_led;
            }

            //  PID gain
            if(pid_gain_msg.was_updated())
            {
                for(int i = 0; i < 4; i++)
                {
                    pid_param[i] = PID::ctrl_param_t {
                        pid_gain_msg.data.gains[i].kp,
                        pid_gain_msg.data.gains[i].ki,
                        pid_gain_msg.data.gains[i].kd,
                        pid_gain_msg.data.gains[i].fg,
                    };
                }
            }
        }
        
        if(sub_ctl_serial.update() == 0)
        {
            if(sub_controller_msg.was_updated())
            {
                //  emergency stop
                emergency_stop = sub_controller_msg.data.emergency_switch;

                //  send another board
                shooter_msg.data.all_reload = sub_controller_msg.data.all_reload;
                shooter_msg.data.shooter.num = sub_controller_msg.data.shooter.num;
                shooter_msg.data.shooter.power = sub_controller_msg.data.shooter.power;
                shooter_msg.data.shooter.action = sub_controller_msg.data.shooter.action;

                //  set variable
                received_movement_variable.x = sub_controller_msg.data.movement.x;
                received_movement_variable.y = sub_controller_msg.data.movement.y;
                received_movement_variable.z = sub_controller_msg.data.movement.z;
                movement_mode = sub_controller_msg.data.movement_mode;

                //  face
                emotion_msg.data.face = controller_msg.data.face;

                //  upstream
                upstream_controller_msg.data.emergency_switch = sub_controller_msg.data.emergency_switch;
                upstream_controller_msg.data.all_reload = sub_controller_msg.data.all_reload;
                upstream_controller_msg.data.shooter.action = sub_controller_msg.data.shooter.action;
                upstream_controller_msg.data.shooter.num = sub_controller_msg.data.shooter.num;
                upstream_controller_msg.data.shooter.power = sub_controller_msg.data.shooter.power;
                upstream_controller_msg.data.movement.x = sub_controller_msg.data.movement.x;
                upstream_controller_msg.data.movement.y = sub_controller_msg.data.all_reload;
                upstream_controller_msg.data.movement.z = sub_controller_msg.data.movement.z;
                upstream_controller_msg.data.movement_mode = sub_controller_msg.data.movement_mode;
                upstream_controller_msg.data.face = sub_controller_msg.data.face;

                pc_serial.write(UPSTREAM_CONTROLLER_ID);

                //  send slave
                slave_serial.write(CONTROLLER_TX_ID);
                slave_serial.write(EMOTION_ID);

                //  Toggle LED
                rx_sub_ctl_led = !rx_sub_ctl_led;
                rx_timeout_led = false;
                //  reset timer
                rx_timer.reset();

                //  timestamp
                last_sub_ctl_response = rx_timestamp.read_ms();
            }
        }
        
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


        //  send
        pc_serial.write(FEEDBACK_TX_ID);

        //  Movement mode
        if(movement_mode == 0)
        {
            drive_variable.x = received_movement_variable.x * MOVEMENT_FAST_XY * MAX_XY_SPEED;
            drive_variable.y = received_movement_variable.y * MOVEMENT_FAST_XY * MAX_XY_SPEED;
            drive_variable.z = received_movement_variable.z * MOVEMENT_FAST_THETA * MAX_THETA_SPEED;
        }
        else if (movement_mode == 1) {
            drive_variable.x = received_movement_variable.x * MOVEMENT_NORMAL_XY * MAX_XY_SPEED;
            drive_variable.y = received_movement_variable.y * MOVEMENT_NORMAL_XY * MAX_XY_SPEED;
            drive_variable.z = received_movement_variable.z * MOVEMENT_NORMAL_THETA * MAX_THETA_SPEED;
        }
        else if (movement_mode == 2) {
            drive_variable.x = received_movement_variable.x * MOVEMENT_SLOW_XY * MAX_XY_SPEED;
            drive_variable.y = received_movement_variable.y * MOVEMENT_SLOW_XY * MAX_XY_SPEED;
            drive_variable.z = received_movement_variable.z * MOVEMENT_SLOW_THETA * MAX_THETA_SPEED;
        }
        
        //  increment
        //  x
        if((drive_variable.x - present_drive_variable.x) > 0)
        {
            present_drive_variable.x += MOVEMENT_ACCERALATION;
        }
        else if ((drive_variable.x - present_drive_variable.x) < 0) {
            present_drive_variable.x -= MOVEMENT_ACCERALATION;
        }
        //  y
        if((drive_variable.y - present_drive_variable.y) > 0)
        {
            present_drive_variable.y += MOVEMENT_ACCERALATION;
        }
        else if ((drive_variable.y - present_drive_variable.y) < 0) {
            present_drive_variable.y -= MOVEMENT_ACCERALATION;
        }
        //  theta
        if((drive_variable.z - present_drive_variable.z) > 0)
        {
            present_drive_variable.z += MOVEMENT_ACCERALATION;
        }
        else if ((drive_variable.z - present_drive_variable.z) < 0) {
            present_drive_variable.z -= MOVEMENT_ACCERALATION;
        }

        //  fit
        //  x
        if(abs(drive_variable.x - present_drive_variable.x) < MOVEMENT_FIT_RANGE)
        {
            present_drive_variable.x = drive_variable.x;
        }
        //  y
        if(abs(drive_variable.y - present_drive_variable.y) < MOVEMENT_FIT_RANGE)
        {
            present_drive_variable.y = drive_variable.y;
        }
        //  z
        if(abs(drive_variable.z - present_drive_variable.z) < MOVEMENT_FIT_RANGE)
        {
            present_drive_variable.z = drive_variable.z;
        }

        //  Drive omuni4
        omuni4->drive(
            -1 * present_drive_variable.x, 
            -1 * present_drive_variable.y,
            present_drive_variable.z
        );
                
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
        true
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
        0.2,
        0.0,
        0.0,
        0.125,
        false
    };
    pid_param[1] = PID::ctrl_param_t {
        0.2,
        0.0,
        0.0,
        0.125,
        false
    };
    pid_param[2] = PID::ctrl_param_t {
        0.2,
        0.0,
        0.0,
        0.125,
        false
    };
    pid_param[3] = PID::ctrl_param_t {
        0.2,
        0.0,
        0.0,
        0.125,
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
