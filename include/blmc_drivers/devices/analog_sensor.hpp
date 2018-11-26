/**
 * @file analog_sensor.hpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-23
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#pragma once

#include <memory>
#include <string>

#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <blmc_drivers/devices/device_interface.hpp>
#include <blmc_drivers/devices/motor_board.hpp>


namespace blmc_drivers
{

/**
 * @brief AnalogSensorInterface class is a simple abstract interface for using
 * blmc analog measurements.
 */
class AnalogSensorInterface: public DeviceInterface
{
public:
    /**
     * @brief This is just a short cut for the time series types
     */
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;


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
    virtual ~AnalogSensorInterface() {}
};

/**
 * @brief The AnalogSensorclass is the implementation of the above interface.
 */
class AnalogSensor: public AnalogSensorInterface
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
   * @brief sensor_id_ is the identification number of the sensor on the control
   * board, for now it is either 0 or 1
   */
  size_t sensor_id_;
};

}
