#include "ThisThread.h"
#include "lorawan_types.h"
#include "mbed.h"
#include "mbed_thread.h"

/////////////////////////////
//  Private include

//  Serial Bridge
#include <SerialBridge.hpp>
#include <MbedHardwareSerial.hpp>
#include <Controller.hpp>

/////////////////////////////
//  Private definition

//  LOOP RATE
#define LOOP_RATE 30

//  RX Timeout
#define RX_TIMEOUT 1000

//  LEDs
#define RX_LED A0
#define RX_TIMEOUT_LED A0

//  Serial Bridge
#define CONTROLLER_RX_ID 0
#define CONTROLLER_TX_ID 1

/////////////////////////////
//  Private variable

//  LEDs
DigitalOut rx_led(RX_LED);
DigitalOut rx_timeout_led(RX_TIMEOUT_LED);

//  Timers
//  PID clock timer
Timer pid_timer;
//  RX timeout timer
Timer rx_timer;

//  Serial Bridge
SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
SerialBridge serial(dev, 1024);
//  Serial Bridge Message
Controller controller_msg[2];


int main()
{
    //  start timers
    pid_timer.start();
    rx_timer.start();

    //  Serial Bridge
    serial.add_frame(CONTROLLER_RX_ID, &controller_msg[0]);
    serial.add_frame(CONTROLLER_TX_ID, &controller_msg[1]);

    while (true) {
        //  rx timeout
        if(duration_cast<std::chrono::milliseconds>(rx_timer.elapsed_time()).count() > RX_TIMEOUT)
        {
            // Toggle RX_TIMEOUT LED
            rx_timeout_led = true;

            //  set zero all control value
            controller_msg[0].data.movement.x = 0;
            controller_msg[0].data.movement.y = 0;
            controller_msg[0].data.movement.z = 0;
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

                serial.write(CONTROLLER_TX_ID);

                //  Toggle LED
                rx_led = !rx_led;
                rx_timeout_led = false;
                //  reset timer
                rx_timer.reset();
            }
        }

        // ThisThread::sleep_for(LOOP_RATE);
        thread_sleep_for(LOOP_RATE);
    }
}

