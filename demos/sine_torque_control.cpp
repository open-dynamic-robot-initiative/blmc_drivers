/**
 * @file sine_torque_control.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "sine_torque_control.hpp"
#include <fstream>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace blmc_drivers
{
void SineTorqueControl::loop()
{
    const int& blmc_position_index = MotorInterface::MeasurementIndex::position;
    const int& blmc_velocity_index = MotorInterface::MeasurementIndex::velocity;
    const int& blmc_current_index = MotorInterface::MeasurementIndex::current;
    // some data
    double actual_position = 0.0;
    double actual_velocity = 0.0;
    double actual_current = 0.0;
    double local_time = 0.0;
    // sine torque params
    double current_amplitude = 0.0;
    double current_period = 2.0;
    double current_pulsation = 2 * 3.1415 / current_period;
    // here is the control in current (Ampere)
    double desired_current = 0.0;

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;
    while (!stop_loop_)
    {
        time_logger.tic();
        local_time = count * 0.001;

        // compute the control
        for (unsigned int i = 0; i < motor_list_.size(); ++i)
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

            desired_current =
                current_amplitude * sin(current_pulsation * local_time);

            motor_list_[i]->set_current_target(desired_current);
            motor_list_[i]->send_if_input_changed();

            encoders_[i].push_back(actual_position);
            velocities_[i].push_back(actual_velocity);
            currents_[i].push_back(actual_current);
            control_buffer_[i].push_back(desired_current);

            // we sleep here 1ms.
            spinner.spin();
            // measure the time spent.
            time_logger.tac();

            // Printings
            if ((count % 1000) == 0)
            {
                rt_printf("sending current: %f\n", desired_current);
                // time_logger.print_statistics();
            }
            ++count;
        }  // endfor
    }      // endwhile
    time_logger.dump_measurements("/tmp/demo_pd_control_time_measurement");
}

void SineTorqueControl::stop_loop()
{
    // dumping stuff
    std::string file_name = "/tmp/sine_torque_xp.dat";
    try
    {
        std::ofstream log_file(file_name, std::ios::binary | std::ios::out);
        log_file.precision(10);

        assert(encoders_[0].size() == velocities_[0].size() &&
               velocities_[0].size() == control_buffer_[0].size() &&
               control_buffer_[0].size() == currents_[0].size());
        for (unsigned int j = 0; j < encoders_[0].size(); ++j)
        {
            for (unsigned int i = 0; i < encoders_.size(); ++i)
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