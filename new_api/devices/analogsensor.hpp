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
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const = 0;

    virtual ~AnalogsensorInterface() {}
};

class Analogsensor: public AnalogsensorInterface
{
    std::map<std::string, std::string> sensor_to_board_name_;
    std::string name_;
    std::shared_ptr<CanMotorboard> board_;

public:
    Analogsensor(std::shared_ptr<CanMotorboard> board, bool sensor_id):
        board_(board)
    {
        name_ = "analog_" + std::to_string(sensor_id);
    }

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const
    {
        return board_->measurement(name_);
    }

};
