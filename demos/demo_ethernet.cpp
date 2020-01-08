/**
 * @file demo_ethernet.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck Gesellschaft, License BSD-3-Clause
 */

#include <atomic>
#include <signal.h>

#include "blmc_drivers/devices/spi_motor_board.hpp"
#include "sine_position_control.hpp"

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool g_stop_demo (false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 * 
 * @param s 
 */
void my_handler(int){
  g_stop_demo = true;
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
    g_stop_demo = false;

    // Number of udriver plugged
    int nb_udrivers = 6;

    // First of all one need to initialize the communication with the master
    // board via ethernet. The include for this class comes with
    // #include "blmc_drivers/devices/ethernet_wifi_motor_board.hpp"
    std::shared_ptr<MasterBoardInterface> master_board_interface = 
        std::make_shared<MasterBoardInterface>(network_id);
    master_board_interface->Init();
    std::shared_ptr<blmc_drivers::SpiBus> spi_bus =
        std::make_shared<blmc_drivers::SpiBus>(master_board_interface, nb_udrivers);

    // Then we create a motor board object that will use the SPI bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are aligned during this stage.
    std::vector<std::shared_ptr<blmc_drivers::SpiMotorBoard> > boards;
    boards.resize(nb_udrivers);
    for(size_t i = 0 ; i < boards.size() ; ++i)
    {
        boards[i] = std::make_shared<blmc_drivers::SpiMotorBoard>(spi_bus, i);
    }

    // create the motors object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    std::vector<blmc_drivers::SafeMotor_ptr> motor_list;
    motor_list.resize(nb_udrivers * 2);
    for(size_t i = 0 ; i < boards.size() ; ++i)
    {
        motor_list[2*i] = std::make_shared<blmc_drivers::SafeMotor>(boards[i], 0, 1.0);
        motor_list[2*i + 1] = std::make_shared<blmc_drivers::SafeMotor>(boards[i], 1, 1.0);
    }
    
    rt_printf("motors are set up \n");

    // construct a simple PD controller following a sinus trajectory.
    blmc_drivers::SinePositionControl controller(motor_list);
    controller.set_gains(/*kp=*/2.0, /*kd=*/0.1);

    rt_printf("controllers are set up \n");

    spi_bus->wait_until_ready();
    controller.start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while(!g_stop_demo)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }
    return 0;
}
