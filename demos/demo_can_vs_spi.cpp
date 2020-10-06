/**
 * @file demo_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <array>
#include <deque>
#include <iostream>
#include <fstream>

#include "blmc_drivers/devices/analog_sensor.hpp"
#include "blmc_drivers/devices/can_bus.hpp"
#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/devices/motor_board.hpp"
#include "blmc_drivers/devices/spi_motor_board.hpp"
#include "real_time_tools/mutex.hpp"

#define NB_MOTOR 12
#define LOG_SIZE 300000

struct Hardware
{
    std::shared_ptr<blmc_drivers::CanBusInterface> can_bus;
    std::shared_ptr<blmc_drivers::MotorBoardInterface> can_motor_board;
    std::shared_ptr<blmc_drivers::Motor> can_motor;

    std::shared_ptr<MasterBoardInterface> master_board_interface;

    std::array<double, LOG_SIZE> time;
    std::array<double, LOG_SIZE> can_velocity;
    std::array<double, LOG_SIZE> spi_velocity;

    real_time_tools::RealTimeMutex mutex;

    std::atomic_bool running;
};

static THREAD_FUNCTION_RETURN_TYPE control_loop(void *hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware &hardware = *(static_cast<Hardware *>(hardware_ptr));

    // Control parameter
    int log_index = 0;
    double dt = 0.001;
    double t = 0;
    double kp = 2.0;
    double kd = 0.05;
    double iq_sat = 4.0;
    double freq = 0.5;
    double amplitude = M_PI;
    double init_pos[N_SLAVES * 2] = {0};
    int state = 0;
    const int &blmc_velocity_index =
        blmc_drivers::MotorInterface::MeasurementIndex::velocity;

    // torque controller -------------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(dt);

    // Main loop
    while (!hardware.master_board_interface->IsTimeout() &&
           log_index < LOG_SIZE)
    {
        t += dt;
        hardware.master_board_interface
            ->ParseSensorData();  // This will read the last incomming
                                  // packet and update all sensor fields.
        switch (state)
        {
            case 0:  // check the end of calibration (are the all controlled
                     // motor enabled and ready?)
                state = 1;
                for (int i = 0; i < NB_MOTOR; i++)
                {
                    if (!(hardware.master_board_interface->motors[i]
                              .IsEnabled() &&
                          hardware.master_board_interface->motors[i].IsReady()))
                    {
                        state = 0;
                    }
                    init_pos[i] = hardware.master_board_interface->motors[i]
                                      .GetPosition();  // initial position
                    t = 0;                             // to start sin at 0
                }
                break;
            case 1:
                // Closed loop position and data acquisition.
                for (int i = 0; i < NB_MOTOR; i++)
                {
                    if (hardware.master_board_interface->motors[i].IsEnabled())
                    {
                        double ref =
                            init_pos[i] + amplitude * sin(2 * M_PI * freq * t);
                        double v_ref = 2. * M_PI * freq * amplitude *
                                       cos(2 * M_PI * freq * t);
                        double p_err =
                            ref - hardware.master_board_interface->motors[i]
                                      .GetPosition();
                        double v_err =
                            v_ref - hardware.master_board_interface->motors[i]
                                        .GetVelocity();
                        double cur = kp * p_err + kd * v_err;
                        if (cur > iq_sat) cur = iq_sat;
                        if (cur < -iq_sat) cur = -iq_sat;
                        hardware.master_board_interface->motors[i]
                            .SetCurrentReference(cur);
                    }
                }

                hardware.time[log_index] = t;
                hardware.can_velocity[log_index] =
                    hardware.can_motor->get_measurement(blmc_velocity_index)
                        ->newest_element();
                hardware.spi_velocity[log_index] =
                    hardware.master_board_interface->motors[0].GetVelocity();

                ++log_index;
                break;
        }
        hardware.master_board_interface
            ->SendCommand();  // This will send the command packet
        // Spin for the rest of the control loop.
        spinner.spin();
    }

    if (hardware.master_board_interface->IsTimeout())
    {
        printf(
            "Masterboard timeout detected. Either the masterboard has been "
            "shut down or there has been a connection issue with the "
            "cable/wifi.\n");
    }
    hardware.running = false;
    return THREAD_FUNCTION_RETURN_VALUE;
}

static THREAD_FUNCTION_RETURN_TYPE printing_loop(void *hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware &hardware = *(static_cast<Hardware *>(hardware_ptr));

    // print loop parameters
    int cpt = 0;

    // print info --------------------------------------------------------------
    long int timeindex =
        hardware.can_bus->get_output_frame()->newest_timeindex();

    while (hardware.running)
    {
        ++cpt;

        if (cpt % 100 == 0)
        {
            cpt = 0;
            rt_printf("\33[H\33[2J");  // clear screen
            hardware.master_board_interface->PrintIMU();
            hardware.master_board_interface->PrintADC();
            hardware.master_board_interface->PrintMotors();
            hardware.master_board_interface->PrintMotorDrivers();

            long int received_timeindex = timeindex;
            // this will return the element with the index received_timeindex,
            // if this element does not exist anymore, it will return the oldest
            // element it still has and change received_timeindex to the
            // appropriate index.
            blmc_drivers::CanBusFrame can_frame =
                (*hardware.can_bus->get_output_frame())[received_timeindex];
            timeindex++;

            rt_printf("timeindex: %ld\n", timeindex);
            can_frame.print();

            fflush(stdout);
        }
    }
    return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int argc, char **argv)
{
    // get the arguments
    if (argc != 2)
    {
        throw std::runtime_error("Please provide the network id as argument.");
    }
    std::string network_id = argv[1];

    // create the hardware pointers.
    Hardware hardware;

    // Used to kill the printing thread.
    hardware.running = true;

    // Initialize the TI card through the can bus.
    hardware.can_bus = std::make_shared<blmc_drivers::CanBus>("can0");
    hardware.can_motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(hardware.can_bus);
    hardware.can_motor =
        std::make_shared<blmc_drivers::SafeMotor>(hardware.can_motor_board, 0);

    // Initialize the communication with the master_board.
    hardware.master_board_interface =
        std::make_shared<MasterBoardInterface>(network_id);
    hardware.master_board_interface->Init();
    // Initialisation, send the init commands
    for (int i = 0; i < 6; i++)
    {
        hardware.master_board_interface->motor_drivers[i]
            .motor1->SetCurrentReference(0);
        hardware.master_board_interface->motor_drivers[i]
            .motor2->SetCurrentReference(0);
        hardware.master_board_interface->motor_drivers[i].motor1->Enable();
        hardware.master_board_interface->motor_drivers[i].motor2->Enable();
        hardware.master_board_interface->motor_drivers[i]
            .EnablePositionRolloverError();
        hardware.master_board_interface->motor_drivers[i].SetTimeout(5);
        hardware.master_board_interface->motor_drivers[i].Enable();
    }

    // start real-time control loop
    // --------------------------------------------
    real_time_tools::RealTimeThread control_thread;
    control_thread.create_realtime_thread(&control_loop, &hardware);

    // start printing loop -------------------------------------------
    std::thread printing_thread(&printing_loop, &hardware);

    rt_printf("control loop started \n");
    fflush(stdout);
    control_thread.join();
    printing_thread.join();

    rt_printf("control loop stopped \n");
    rt_printf("logging data \n");
    fflush(stdout);

    std::ofstream myfile;
    myfile.open ("/tmp/can_velocity.dat");
    for(unsigned int i=0 ; i<LOG_SIZE ; ++i)
    {
        myfile << i << " " << hardware.can_velocity[i] << std::endl;
    }
    myfile.close();

    myfile.open ("/tmp/spi_velocity.dat");
    for(unsigned int i=0 ; i<LOG_SIZE ; ++i)
    {
        myfile << i << " " << hardware.can_velocity[i] << std::endl;
    }
    myfile.close();

    return 0;
}
