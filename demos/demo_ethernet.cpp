/**
 * @file demo_3_motors.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-29
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <atomic>
#include <signal.h>

#include "blmc_drivers/devices/ethernet_wifi_motor_board.hpp"
#include "sine_position_control.hpp"

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos (false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 * 
 * @param s 
 */
void my_handler(int){
  StopDemos = true;
  printf("Stopping demo.\n");
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
    // get the arguments
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the network id as argument.");
    }
    std::string network_id = argv[1];

    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    // First of all one need to initialize the communication with the master
    // board via ethernet. The include for this class comes with
    // #include "blmc_drivers/devices/ethernet_wifi_motor_board.hpp"
    std::shared_ptr<MasterBoardInterface> master_board_interface = 
        std::make_shared<MasterBoardInterface>(network_id);
    master_board_interface->Init();

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    auto board1 = std::make_shared<blmc_drivers::EthernetWifiMotorBoard>(
      master_board_interface, 0);
    auto board2 = std::make_shared<blmc_drivers::EthernetWifiMotorBoard>(
      master_board_interface, 1);

    // create the motors object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    std::vector<blmc_drivers::SafeMotor_ptr> motor_list; 
    motor_list.emplace_back(std::make_shared<blmc_drivers::SafeMotor>(board1, 0));
    motor_list.emplace_back(std::make_shared<blmc_drivers::SafeMotor>(board1, 1));
    motor_list.emplace_back(std::make_shared<blmc_drivers::SafeMotor>(board2, 0));

    blmc_drivers::SafeMotor motor_idle(board2, 1);
    motor_idle.set_current_target(0.0);

    rt_printf("motors are set up \n");

    // construct a simple PD controller following a sinus trajectory.
    blmc_drivers::SinePositionControl controller(motor_list,
                                                 master_board_interface); 

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
