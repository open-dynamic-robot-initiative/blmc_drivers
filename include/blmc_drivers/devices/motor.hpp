/**
 * @file motor.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019-2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <memory>
#include <string>

#include <real_time_tools/timer.hpp>
#include <time_series/time_series.hpp>

#include "blmc_drivers/devices/device_interface.hpp"
#include "blmc_drivers/devices/motor_board.hpp"

namespace blmc_drivers
{
/**
 * @brief This class declares an interface to the motor. It allows the user to
 * access the sensors data as well as sending controls. The only control
 * supported for now is the current.
 */
class MotorInterface : public DeviceInterface
{
public:
    /**
     * @brief This is a useful alias.
     */
    typedef time_series::TimeSeries<double> ScalarTimeseries;
    /**
     * @brief This a useful alias for the shared Pointer creation.
     *
     * @tparam Type is the Class to crate the pointer from.
     */
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;

    /**
     * @brief Here is a list of the different measurement available on the
     * blmc card.
     */
    enum MeasurementIndex
    {
        current,
        position,
        velocity,
        encoder_index,
        measurement_count
    };

    /**
     * @brief Destroy the MotorInterface object
     */
    virtual ~MotorInterface()
    {
    }

    /**
     * @brief Actually send the commands and controls.
     */
    virtual void send_if_input_changed() = 0;

    /**
     * Getters
     */

    /**
     * @brief Get the measurements.
     *
     * @param index
     * @return Ptr<const ScalarTimeseries> the pointer to the desired
     * measurement history.
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(
        const int& index = 0) const = 0;

    /**
     * @brief Get the current target object
     *
     * @return Ptr<const ScalarTimeseries> the list of the current values to
     * be sent.
     */
    virtual Ptr<const ScalarTimeseries> get_current_target() const = 0;

    /**
     * @brief Get the history of the sent current targets.
     *
     * @return Ptr<const ScalarTimeseries>
     */
    virtual Ptr<const ScalarTimeseries> get_sent_current_target() const = 0;

    /**
     * Setters
     */

    /**
     * @brief Set the current target. This function saves the data internally.
     * Please call send_if_input_changed() to actually send the data.
     *
     * @param current_target
     */
    virtual void set_current_target(const double& current_target) = 0;

    /**
     * @brief Set the command. Save internally a command to be apply by the
     * motor board. This function save the command internally. Please call
     * send_if_input_changed() to actually send the data.
     *
     * @param command
     */
    virtual void set_command(const MotorBoardCommand& command) = 0;
};

/**
 * @brief This class implements the MotorInterface.
 */
class Motor : public MotorInterface
{
public:
    /**
     * @brief Construct a new Motor object
     *
     * @param board is the MotorBoard to be used.
     * @param motor_id is the id of the motor on the on-board card
     */
    Motor(Ptr<MotorBoardInterface> board, bool motor_id);

    /**
     * @brief Destroy the Motor object
     *
     */
    virtual ~Motor()
    {
    }

    /**
     * @brief Actually send the command and controls via the network,
     * See MotorInterface for more information.
     */
    virtual void send_if_input_changed()
    {
        board_->send_if_input_changed();
    }

    /**
     * Getters
     */

    /**
     * @brief Get the measurements
     *
     * @param index is the kind of measurement we are instersted in.
     * see MotorInterface::MeasurementIndex.
     * @return Ptr<const ScalarTimeseries> The history of the measurement
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(
        const int& index = 0) const;

    /**
     * @brief Get the current target to be sent.
     *
     * @return Ptr<const ScalarTimeseries> the list of current values to be
     * sent.
     */
    virtual Ptr<const ScalarTimeseries> get_current_target() const;

    /**
     * @brief Get the already sent current target values.
     *
     * @return Ptr<const ScalarTimeseries>
     */
    virtual Ptr<const ScalarTimeseries> get_sent_current_target() const;

    /**
     * Setters
     */

    /**
     * @brief Set the current (Ampere) target. See MotorInterface for more
     * information.
     *
     * @param current_target in Ampere
     */
    virtual void set_current_target(const double& current_target);

    /**
     * @brief Set the command. See MotorInterface for more information.
     *
     * @param command
     */
    virtual void set_command(const MotorBoardCommand& command)
    {
        board_->set_command(command);
    }

    /** @brief Print the motor status and state. */
    virtual void print() const;

protected:
    /**
     * @brief The MotorBoard to be used for the communication.
     */
    Ptr<MotorBoardInterface> board_;

    /**
     * @brief The id of the motor on the MotorBoard.
     */
    bool motor_id_;
};

/**
 * @brief This class is a safe implementation of the Motor class.
 * It contains utilities to bound the control input.
 * It could also contains some velocity limits at the motor level and why not
 * some temperature management.
 *
 * \todo the velocity limit should be implemented in a smoother way,
 * and the parameters should be passed in the constructor.
 */
class SafeMotor : public Motor
{
public:
    /**
     * @brief Construct a new SafeMotor object
     *
     * @param board
     * @param motor_id
     * @param max_current_target
     * @param history_length
     */
    SafeMotor(
        Ptr<MotorBoardInterface> board,
        bool motor_id,
        const double& max_current_target = 2.0,
        const size_t& history_length = 1000,
        const double& max_velocity = std::numeric_limits<double>::quiet_NaN());

    /**
     * Getters
     */

    /**
     * @brief Get the _current_target object
     *
     * @return Ptr<const ScalarTimeseries>
     */
    virtual Ptr<const ScalarTimeseries> get_current_target() const
    {
        return current_target_;
    }

    /**
     * Setters
     */

    /**
     * @brief Set the current target (Ampere)
     *
     * @param current_target
     */
    virtual void set_current_target(const double& current_target);

    /**
     * @brief Set the max_current_target_ object
     *
     * @param max_current_target
     */
    void set_max_current(double max_current_target)
    {
        max_current_target_ = max_current_target;
    }

    /**
     * @brief Set the max_velocity_ constant.
     *
     * @param max_velocity
     */
    void set_max_velocity(double max_velocity)
    {
        max_velocity_ = max_velocity;
    }

private:
    /**
     * @brief max_current_target_ is the limit of the current.
     */
    double max_current_target_;

    /**
     * @brief max_velocity_ limits the motor velocity.
     */
    double max_velocity_;

    /**
     * @brief History of the target current sent.
     */
    Ptr<ScalarTimeseries> current_target_;
};

}  // namespace blmc_drivers
