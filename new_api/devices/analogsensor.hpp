#pragma once

#include <blmc_can/blmc_can.h>

#include <memory>
#include <string>

#include <utils/timer.hpp>
#include <utils/threadsafe_object.hpp>
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
    std::map<std::string, std::string> sensor_to_board_name_;
    std::string name_;

    enum Measurement {analog, measurement_count};

    std::vector<std::shared_ptr<const ScalarTimeseries>> measurement_;

    std::shared_ptr<CanMotorboard> board_;

public:
    Analogsensor(std::shared_ptr<CanMotorboard> board, bool sensor_id):
        board_(board)
    {
        measurement_.resize(measurement_count);
        if(sensor_id == 0)
        {
            measurement_[analog] = board_->measurement(CanMotorboard::analog_0);
        }
        else
        {
            measurement_[analog] = board_->measurement(CanMotorboard::analog_1);
        }
    }

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const
    {
        return measurement_[analog];
    }

};
