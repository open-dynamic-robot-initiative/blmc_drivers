#pragma once

#include <memory>
#include <string>

#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <blmc_drivers/devices/device_interface.hpp>
#include <blmc_drivers/devices/motor_board.hpp>

class AnalogSensorInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;

    virtual std::shared_ptr<const ScalarTimeseries> get_measurement() const = 0;

    virtual ~AnalogSensorInterface() {}
};

class AnalogSensor: public AnalogSensorInterface
{
    std::shared_ptr<MotorBoardInterface> board_;
    size_t sensor_id_;

public:
    AnalogSensor(std::shared_ptr<MotorBoardInterface> board, bool sensor_id):
        board_(board),
        sensor_id_(sensor_id)
    { }

    virtual std::shared_ptr<const ScalarTimeseries> get_measurement() const
    {
        if(sensor_id_ == 0)
        {
            return board_->get_measurement(MotorBoardInterface::analog_0);
        }
        else
        {
            return board_->get_measurement(MotorBoardInterface::analog_1);
        }
    }

};
