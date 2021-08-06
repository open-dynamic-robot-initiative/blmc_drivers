/**
 * @file analog_sensor.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <memory>
#include <string>

#include <time_series/time_series.hpp>
#include "real_time_tools/timer.hpp"

#include "blmc_drivers/devices/device_interface.hpp"
#include "blmc_drivers/devices/motor_board.hpp"

namespace blmc_drivers
{
/**
 * @brief AnalogSensorInterface class is a simple abstract interface for using
 * blmc analog measurements.
 */
class AnalogSensorInterface : public DeviceInterface
{
public:
    /**
     * @brief This is just a short cut for the time series types
     */
    typedef time_series::TimeSeries<double> ScalarTimeseries;

    /**
     * @brief Get the measurement object which is the list of time stamped data.
     *
     * @return std::shared_ptr<const ScalarTimeseries> which is a pointer to the
     * a list of time stamped data
     */
    virtual std::shared_ptr<const ScalarTimeseries> get_measurement() const = 0;

    /**
     * @brief Destroy the AnalogSensorInterface object
     *
     */
    virtual ~AnalogSensorInterface()
    {
    }
};

/**
 * @brief AnalogSensor class is the implementation of the above interface.
 */
class AnalogSensor : public AnalogSensorInterface
{
public:
    /**
     * @brief Construct a new AnalogSensor object
     *
     * @param board is a motor board which gives access to the motor sensors
     * (position, velocity, current, etc) and to the motor cotrols.
     * @param sensor_id is the id of the sensor on the control board
     */
    AnalogSensor(std::shared_ptr<MotorBoardInterface> board, bool sensor_id);

    /**
     * @brief Get the measurement object which is the list of time stamped data.
     *
     * @return std::shared_ptr<const ScalarTimeseries> which is a pointer to the
     * a list of time stamped data
     */
    virtual std::shared_ptr<const ScalarTimeseries> get_measurement() const;

private:
    /**
     * @brief board_ is the measurement object, it caontains the list
     * of the timed stamped data
     */
    std::shared_ptr<MotorBoardInterface> board_;

    /**
     * @brief sensor_id_ is the identification number of the sensor on the
     * control board, for now it is either 0 or 1
     */
    size_t sensor_id_;
};

}  // namespace blmc_drivers
