/**
 * @file demo_3_motors.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck Gesellschaft, License BSD-3-Clause
 */

#include <atomic>
#include <signal.h>

#include "pd_control.hpp"

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos (false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 * 
 * @param s 
 */
void my_handler(int s){
  StopDemos = true;
}

/**
 * @brief This is the main demo program.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{   
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    // First of all one need to initialize the communication with the can bus.
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can0");
    auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("can1");

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    auto board1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
    auto board2 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);

    // create the motors object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    auto motor_1 = std::make_shared<blmc_drivers::SafeMotor>(board1, 0);
    auto motor_2 = std::make_shared<blmc_drivers::SafeMotor>(board1, 1);
    auto motor_3 = std::make_shared<blmc_drivers::SafeMotor>(board2, 0);

    rt_printf("motors are set up \n");

    // create 2 analogue sensors onject which happen to be slider here, i.e.
    // 2 linear potentiometers.
    auto slider_1 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 0);
    auto slider_2 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 1);
    auto slider_3 = std::make_shared<blmc_drivers::AnalogSensor>(board2, 0);

    rt_printf("sensors are set up \n");

    // construct the pairs of motors and sliders
    std::vector<blmc_drivers::PairMotorSlider> motor_and_sliders;
    motor_and_sliders.push_back(
      blmc_drivers::PairMotorSlider(motor_1, slider_1));
    motor_and_sliders.push_back(
      blmc_drivers::PairMotorSlider(motor_2, slider_2));
    motor_and_sliders.push_back(
      blmc_drivers::PairMotorSlider(motor_3, slider_3));

    // construct a simple PD controller.
    blmc_drivers::PDController controller(motor_and_sliders); 

    rt_printf("controllers are set up \n");

    controller.start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while(!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }
    return 0;
}
