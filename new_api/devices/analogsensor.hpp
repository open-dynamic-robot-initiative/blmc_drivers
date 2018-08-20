#pragma once

#include <blmc_can/blmc_can.h>

#include <memory>
#include <string>

#include <utils/timer.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <devices/motorboard.hpp>

class AnalogsensorInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const = 0;

    virtual ~AnalogsensorInterface() {}
};

class Analogsensor: public AnalogsensorInterface
{
    std::shared_ptr<MotorboardInterface> board_;
    size_t sensor_id_;

public:
    Analogsensor(std::shared_ptr<MotorboardInterface> board, bool sensor_id):
        board_(board),
        sensor_id_(sensor_id)
    { }

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const
    {
        if(sensor_id_ == 0)
        {
            return board_->measurement(MotorboardInterface::analog_0);
        }
        else
        {
            return board_->measurement(MotorboardInterface::analog_1);
        }
    }

};
