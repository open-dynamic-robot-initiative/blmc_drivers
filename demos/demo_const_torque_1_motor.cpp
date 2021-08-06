/**
 * @file demo_const_torque_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <signal.h>
#include <atomic>

#include "const_torque_control.hpp"

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int)
{
    StopDemos = true;
}

/**
 * @brief This is the main demo program.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int, char **)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    // First of all one need to initialize the communication with the can bus.
    auto can_bus = std::make_shared<blmc_drivers::CanBus>("can0");

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    auto board = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus);

    // create the motors object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    auto motor_1 = std::make_shared<blmc_drivers::SafeMotor>(board, 0);
    auto motor_2 = std::make_shared<blmc_drivers::SafeMotor>(board, 1);

    rt_printf("motors are set up \n");

    // construct the pairs of motors and sliders
    std::vector<blmc_drivers::SafeMotor_ptr> motor_list;
    motor_list.push_back(motor_1);
    motor_list.push_back(motor_2);

    // construct a simple PD controller.
    blmc_drivers::ConstTorqueControl controller(motor_list);

    rt_printf("controllers are set up \n");

    controller.start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    controller.stop_loop();

    return 0;
}
