/**
 * @file demo_1_motor_print_everything.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <tuple>

#include "blmc_drivers/devices/analog_sensor.hpp"
#include "blmc_drivers/devices/can_bus.hpp"
#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/devices/motor_board.hpp"

typedef std::tuple<std::shared_ptr<blmc_drivers::MotorInterface>,
                   std::shared_ptr<blmc_drivers::AnalogSensorInterface>>
    MotorAndSlider;

struct Hardware
{
    std::shared_ptr<blmc_drivers::CanBusInterface> can_bus;
    std::shared_ptr<blmc_drivers::MotorBoardInterface> motor_board;
    std::shared_ptr<blmc_drivers::MotorInterface> motor;
    std::shared_ptr<blmc_drivers::AnalogSensorInterface> slider;
};

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware& hardware = *(static_cast<Hardware*>(hardware_ptr));

    // torque controller -------------------------------------------------------
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    while (true)
    {
        // The sliders are giving values between 0.0 and 1.0
        // double slider_position =
        //         hardware.slider->get_measurement()->newest_element();

        // We transform it into a current between -2.0 and 2.0
        // double desired_current = (slider_position - 0.5) * 4.0;

        // We send the current to the motor
        hardware.motor->set_current_target(/*desired_current*/ 0.0);
        hardware.motor->send_if_input_changed();

        spinner.spin();
    }
}

static THREAD_FUNCTION_RETURN_TYPE printing_loop(void* hardware_ptr)
{
    // cast input arguments to the right format --------------------------------
    Hardware& hardware = *(static_cast<Hardware*>(hardware_ptr));

    // print info --------------------------------------------------------------
    long int timeindex =
        hardware.can_bus->get_output_frame()->newest_timeindex();

    while (true)
    {
        long int received_timeindex = timeindex;
        // this will return the element with the index received_timeindex,
        // if this element does not exist anymore, it will return the oldest
        // element it still has and change received_timeindex to the appropriate
        // index.
        blmc_drivers::CanBusFrame can_frame =
            (*hardware.can_bus->get_output_frame())[received_timeindex];
        timeindex++;

        rt_printf("timeindex: %ld\n", timeindex);
        can_frame.print();
    }
    return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int, char**)
{
    Hardware hardware;
    // First of all one need to initialize the communication with the can bus.
    hardware.can_bus = std::make_shared<blmc_drivers::CanBus>("can0");

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    hardware.motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(hardware.can_bus);

    // create the motor object that have an index that define the port on which
    // they are plugged on the motor board. This object takes also a MotorBoard
    // object to be able to get the sensors and send the control consistantly.
    // These safe motors have the ability to bound the current that is given
    // as input.
    hardware.motor =
        std::make_shared<blmc_drivers::SafeMotor>(hardware.motor_board, 0);

    // create analogue sensors object which happens to be slider here
    hardware.slider =
        std::make_shared<blmc_drivers::AnalogSensor>(hardware.motor_board, 0);

    // start real-time control loop --------------------------------------------
    real_time_tools::RealTimeThread control_thread;
    control_thread.create_realtime_thread(&control_loop, &hardware);

    // start real-time printing loop -------------------------------------------
    real_time_tools::RealTimeThread printing_thread;
    printing_thread.create_realtime_thread(&printing_loop, &hardware);

    rt_printf("control loop started \n");
    control_thread.join();
    printing_thread.join();
    return 0;
}
