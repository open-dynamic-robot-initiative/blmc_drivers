/**
 * @file pd_control.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "pd_control.hpp"
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace blmc_drivers
{
void PDController::loop()
{
    const int& blmc_position_index = MotorInterface::MeasurementIndex::position;
    const int& blmc_velocity_index = MotorInterface::MeasurementIndex::velocity;
    double slider_pose = 0.0;
    double desired_pose = 0.0;
    double actual_pose = 0.0;
    double actual_velocity = 0.0;
    double kp = 5;
    double kd = 1;
    // here is the control in current (Amper)
    double desired_current = 0.0;

    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;
    while (!stop_loop)
    {
        time_logger.tic();

        // compute the control
        for (std::vector<PairMotorSlider>::iterator pair_it =
                 motor_slider_pairs_.begin();
             pair_it != motor_slider_pairs_.end();
             ++pair_it)
        {
            SafeMotor_ptr motor = pair_it->first;
            Slider_ptr slider = pair_it->second;

            slider_pose = slider->get_measurement()->newest_element();
            // The sliders are giving values between 0.0 and 1.0.
            desired_pose = (slider_pose - 0.5);
            actual_pose =
                motor->get_measurement(blmc_position_index)->newest_element();
            actual_velocity =
                motor->get_measurement(blmc_velocity_index)->newest_element();

            desired_current =
                kp * (desired_pose - actual_pose) - kd * (actual_velocity);

            if (desired_current > 1.0)
            {
                desired_current = 1.0;
            }
            else if (desired_current < -1.0)
            {
                desired_current = -1.0;
            }

            motor->set_current_target(desired_current);
            motor->send_if_input_changed();

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

}  // namespace blmc_drivers