#pragma once

#include <memory>
#include <string>
#include <map>

#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/device_interface.hpp>

namespace blmc_drivers
{

class LegInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    template<typename Type> using Ptr = std::shared_ptr<Type>;

    enum MotorMeasurementIndexing {current, position, velocity, encoder_index,
                                   motor_measurement_count};
    enum MotorIndexing {hip, knee, motor_count};

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    get_motor_measurement(const int& motor_index,
                      const int& measurement_index) const = 0;

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    get_current_target(const int& motor_index) const = 0;
    virtual Ptr<const ScalarTimeseries>
    get_sent_current_target(const int& motor_index) const = 0;

    /// setters ================================================================
    virtual void set_current_target(const double& current_target,
                                    const int& motor_index) = 0;

    /// sender =================================================================
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~LegInterface() {}
};


class Leg: public LegInterface
{
private:
    std::array<std::shared_ptr<MotorInterface>, 2> motors_;

public:
    Leg(std::shared_ptr<MotorInterface> hip_motor,
           std::shared_ptr<MotorInterface> knee_motor)
   
    {
        motors_[hip] = hip_motor;
        motors_[knee] = knee_motor;
    }

    virtual ~Leg() {}

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    get_motor_measurement(const int& motor_index,
                      const int& measurement_index) const
    {
        return motors_[motor_index]->get_measurement(measurement_index);
    }

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    get_current_target(const int& motor_index) const
    {
        return motors_[motor_index]->get_current_target();
    }
    virtual Ptr<const ScalarTimeseries>
    get_sent_current_target(const int& motor_index) const
    {
        return motors_[motor_index]->get_sent_current_target();
    }

    /// setters ================================================================
    virtual void set_current_target(const double& current_target,
                                    const int& motor_index)
    {
        motors_[motor_index]->set_current_target(current_target);
    }

    /// sender =================================================================
    virtual void send_if_input_changed()
    {
        for(size_t i = 0; i < motors_.size(); i++)
            motors_[i]->send_if_input_changed();
    }

    /// ========================================================================
};

}