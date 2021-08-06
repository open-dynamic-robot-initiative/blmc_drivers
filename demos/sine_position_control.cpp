/**
 * @file sine_position_control.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "sine_position_control.hpp"
#include <fstream>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace blmc_drivers
{
void SinePositionControl::loop()
{
    const int& blmc_position_index = MotorInterface::MeasurementIndex::position;
    const int& blmc_velocity_index = MotorInterface::MeasurementIndex::velocity;
    const int& blmc_current_index = MotorInterface::MeasurementIndex::current;
    // some data
    double actual_position = 0.0;
    double actual_velocity = 0.0;
    double actual_current = 0.0;
    double local_time = 0.0;
    double control_period = 0.001;
    // sine torque params
    double amplitude = 0.0 /*3.1415*/;
    double frequence = 0.5;
    // here is the control in current (Ampere)
    double desired_position = 0.0;
    double desired_velocity = 0.0;
    double desired_current = 0.0;

    real_time_tools::Spinner spinner;
    spinner.set_period(control_period);  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;
    while (!stop_loop_)
    {
        time_logger.tic();
        local_time = count * control_period;

        // compute the control
        for (size_t i = 0; i < motor_list_.size(); ++i)
        {
            actual_position = motor_list_[i]
                                  ->get_measurement(blmc_position_index)
                                  ->newest_element();
            actual_velocity = motor_list_[i]
                                  ->get_measurement(blmc_velocity_index)
                                  ->newest_element();
            actual_current = motor_list_[i]
                                 ->get_measurement(blmc_current_index)
                                 ->newest_element();

            desired_position =
                amplitude * sin(2 * M_PI * frequence * local_time);
            desired_velocity = 0.0 /* 2 * M_PI * frequence * amplitude *
                                cos(2 * M_PI * frequence * local_time)*/
                ;
            desired_current = kp_ * (desired_position - actual_position) +
                              kd_ * (desired_velocity - actual_velocity);
            motor_list_[i]->set_current_target(desired_current);
        }
        // Send the controls and log stuff

        for (size_t i = 0; i < motor_list_.size(); ++i)
        {
            motor_list_[i]->send_if_input_changed();

            encoders_[i].push_back(actual_position);
            velocities_[i].push_back(actual_velocity);
            currents_[i].push_back(actual_current);
            control_buffer_[i].push_back(desired_current);
        }

        // we sleep here 1ms.
        spinner.spin();
        // measure the time spent.
        time_logger.tac();

        // Printings
        if ((count % (int)(0.2 / control_period)) == 0)
        {
            rt_printf("\33[H\33[2J");  // clear screen
            for (size_t i = 0; i < motor_list_.size(); ++i)
            {
                rt_printf("des_pose: %8f ; ", desired_position);
                motor_list_[i]->print();
            }
            time_logger.print_statistics();
            fflush(stdout);
        }
        ++count;

    }  // endwhile
    time_logger.dump_measurements("/tmp/demo_pd_control_time_measurement");
}

void SinePositionControl::stop_loop()
{
    // dumping stuff
    std::string file_name = "/tmp/sine_position_xp.dat";
    try
    {
        std::ofstream log_file(file_name, std::ios::binary | std::ios::out);
        log_file.precision(10);

        assert(encoders_[0].size() == velocities_[0].size() &&
               velocities_[0].size() == control_buffer_[0].size() &&
               control_buffer_[0].size() == currents_[0].size());
        for (size_t j = 0; j < encoders_[0].size(); ++j)
        {
            for (size_t i = 0; i < encoders_.size(); ++i)
            {
                log_file << encoders_[i][j] << " " << velocities_[i][j] << " "
                         << control_buffer_[i][j] << " " << currents_[i][j]
                         << " ";
            }
            log_file << std::endl;
        }

        log_file.flush();
        log_file.close();
    }
    catch (...)
    {
        rt_printf(
            "fstream Error in dump_tic_tac_measurements(): "
            "no time measurment saved\n");
    }

    rt_printf("dumped the trajectory");
}

}  // namespace blmc_drivers