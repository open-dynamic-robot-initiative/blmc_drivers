/**
 * @file leg.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include <time_series/time_series.hpp>

#include <blmc_drivers/devices/device_interface.hpp>
#include <blmc_drivers/devices/motor.hpp>

namespace blmc_drivers
{
/**
 * @brief This class defines an interface to control a leg.
 * This legg is composed of 2 motor, one for the hip and one for the knee.
 */
class LegInterface : public DeviceInterface
{
public:
    /**
     * @brief ScalarTimeseries is a simple shortcut for more intelligible code.
     */
    typedef time_series::TimeSeries<double> ScalarTimeseries;

    /**
     * @brief This is a shortcut for creating shared pointer in a simpler
     * writting expression.
     *
     * @tparam Type is the template paramer of the shared pointer.
     */
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

    /**
     * @brief MotorMeasurementIndexing this enum allow to access the different
     * kind of sensor measurements in an understandable way in the code.
     */
    enum MotorMeasurementIndexing
    {
        current,
        position,
        velocity,
        encoder_index,
        motor_measurement_count
    };

    /**
     * @brief This enum list the motors in the leg
     */
    enum MotorIndexing
    {
        hip,
        knee,
        motor_count
    };

    /**
     * @brief Destroy the LegInterface object
     */
    virtual ~LegInterface()
    {
    }

    /**
     * Getters
     */

    /**
     * @brief Get the device output
     *
     * @param[in] motor_index designate the motor from which we want the data
     * from.
     * @param[in] measurement_index is teh kind of data we are looking for.
     * @return Ptr<const ScalarTimeseries>  is the list of the lasts time
     * stamped acquiered.
     */
    virtual Ptr<const ScalarTimeseries> get_motor_measurement(
        const int& motor_index, const int& measurement_index) const = 0;

    /**
     * @brief Get the actual target current
     *
     * @param[in] motor_index designate the motor from which we want the data
     * from.
     * @return Ptr<const ScalarTimeseries> is the list of the lasts time
     * stamped acquiered.
     */
    virtual Ptr<const ScalarTimeseries> get_current_target(
        const int& motor_index) const = 0;

    /**
     * @brief Get the last sent target current.
     *
     * @param[in] motor_index designate the motor from which we want the data
     * from.
     * @return Ptr<const ScalarTimeseries> is the list of the lasts time
     * stamped acquiered.
     */
    virtual Ptr<const ScalarTimeseries> get_sent_current_target(
        const int& motor_index) const = 0;

    /**
     * Setters
     */

    /**
     * @brief Set the current target saves internally the desired current. This
     * data is not send to the motor yet. Please call send_if_input_changed in
     * order to actually send the data to the card.
     *
     * @param current_target is the current to achieve on the motor card.
     * @param motor_index is the motor to control.
     */
    virtual void set_current_target(const double& current_target,
                                    const int& motor_index) = 0;

    /**
     * Sender
     */

    /**
     * @brief Actually send the target current to the motor cards.
     */
    virtual void send_if_input_changed() = 0;
};

/**
 * @brief The leg class is the implementation of the LegInterface. This is
 * the decalartion and the definition of the class as it is very simple.
 */
class Leg : public LegInterface
{
public:
    /**
     * @brief Construct a new Leg object
     *
     * @param hip_motor is the pointer to the hip motor
     * @param knee_motor is the pointer to the knee motor
     */
    Leg(std::shared_ptr<MotorInterface> hip_motor,
        std::shared_ptr<MotorInterface> knee_motor)
    {
        motors_[hip] = hip_motor;
        motors_[knee] = knee_motor;
    }

    /**
     * @brief Destroy the Leg object
     */
    virtual ~Leg()
    {
    }

    /**
     * Getters
     */

    /**
     * @brief Get the motor measurements.
     *
     * @param motor_index
     * @param measurement_index
     * @return Ptr<const ScalarTimeseries>
     */
    virtual Ptr<const ScalarTimeseries> get_motor_measurement(
        const int& motor_index, const int& measurement_index) const
    {
        return motors_[motor_index]->get_measurement(measurement_index);
    }

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_current_target(
        const int& motor_index) const
    {
        return motors_[motor_index]->get_current_target();
    }
    virtual Ptr<const ScalarTimeseries> get_sent_current_target(
        const int& motor_index) const
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
        for (size_t i = 0; i < motors_.size(); i++)
            motors_[i]->send_if_input_changed();
    }

    /// ========================================================================
private:
    /**
     * @brief This list contains pointers to two motors. This motors are
     * respectively the hip and the knee of the leg.
     */
    std::array<std::shared_ptr<MotorInterface>, 2> motors_;
};

}  // namespace blmc_drivers
