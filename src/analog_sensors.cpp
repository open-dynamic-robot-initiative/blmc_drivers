/**
 * @file analog_sensors.cpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file defines a class to get access to analogue sensors.
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include <blmc_drivers/devices/analog_sensor.hpp>

namespace blmc_drivers
{
AnalogSensor::AnalogSensor(std::shared_ptr<MotorBoardInterface> board,
                           bool sensor_id)
    : board_(board), sensor_id_(sensor_id)
{
}

std::shared_ptr<const AnalogSensorInterface::ScalarTimeseries>
AnalogSensor::get_measurement() const
{
    if (sensor_id_ == 0)
    {
        return board_->get_measurement(MotorBoardInterface::analog_0);
    }
    else
    {
        return board_->get_measurement(MotorBoardInterface::analog_1);
    }
}

}  // namespace blmc_drivers