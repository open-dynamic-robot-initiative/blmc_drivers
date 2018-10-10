#pragma once

#include <memory>
#include <string>
#include <map>


#include <utils/threadsafe_object.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <devices/motor.hpp>
#include <devices/device_interface.hpp>



class FingerInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    template<typename Type> using Ptr = std::shared_ptr<Type>;

    enum MeasurementIndex {current, position, velocity, encoder_index,
                                   motor_measurement_count};
    enum MotorIndex {interior, center, tip, motor_count};

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    motor_measurement(const int& motor_index,
                      const int& measurement_index) const = 0;

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    current_target(const int& motor_index) const = 0;
    virtual Ptr<const ScalarTimeseries>
    sent_current_target(const int& motor_index) const = 0;

    /// setters ================================================================
    virtual void set_current_target(const double& current_target,
                                    const int& motor_index) = 0;

    /// sender =================================================================
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~FingerInterface() {}
};


class Finger: public FingerInterface
{
private:
    std::array<std::shared_ptr<MotorInterface>, 3> motors_;

public:
    Finger(std::shared_ptr<MotorInterface> interior_motor,
           std::shared_ptr<MotorInterface> center_motor,
           std::shared_ptr<MotorInterface> tip_motor)
    {
        motors_[interior] = interior_motor;
        motors_[center] = center_motor;
        motors_[tip] = tip_motor;
    }

    virtual ~Finger() {}

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    motor_measurement(const int& motor_index,
                      const int& measurement_index) const
    {
        return motors_[motor_index]->measurement(measurement_index);
    }

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries>
    current_target(const int& motor_index) const
    {
        return motors_[motor_index]->current_target();
    }
    virtual Ptr<const ScalarTimeseries>
    sent_current_target(const int& motor_index) const
    {
        return motors_[motor_index]->sent_current_target();
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

