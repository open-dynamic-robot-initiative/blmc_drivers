/**
 * @file blmc_joint_module.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 * @date 2019-07-11
 */
#pragma once

#include <math.h>
#include <array>
#include <iostream>
#include <stdexcept>

#include <Eigen/Eigen>

#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/utils/polynome.hpp"

namespace blmc_drivers
{
typedef blmc_drivers::MotorInterface::MeasurementIndex mi;

// TODO what is the best scope for those homing-related types?

/**
 * @brief Possible return values of the homing
 */
enum class HomingReturnCode
{
    //! Homing was not initialized and can therefore not be performed.
    NOT_INITIALIZED = 0,
    //! Homing is currently running.
    RUNNING,
    //! Homing is succeeded.
    SUCCEEDED,
    //! Homing failed.
    FAILED
};

/**
 * @brief Possible return values of the go_to
 */
enum class GoToReturnCode
{
    //! GoTo is currently running.
    RUNNING,
    //! Position has been reached succeeded.
    SUCCEEDED,
    //! Robot is stuck(hit an obstacle) before reaching its final position.
    FAILED
};

/**
 * @brief State variables required for the homing.
 */
struct HomingState
{
    //! Id of the joint.  Just used for debug prints.
    int joint_id = 0;
    //! Max. distance to move while searching the encoder index.
    double search_distance_limit_rad = 0.0;
    //! Offset from home position to zero position.
    double home_offset_rad = 0.0;
    //! Step size for the position profile.
    double profile_step_size_rad = 0.0;
    //! Timestamp from when the encoder index was seen the last time.
    long int last_encoder_index_time_index = 0;
    //! Number of profile steps already taken.
    uint32_t step_count = 0;
    //! Current target position of the position profile.
    double target_position_rad = 0.0;
    //! Current status of the homing procedure.
    HomingReturnCode status = HomingReturnCode::NOT_INITIALIZED;

    //! Position at which homing is started
    double start_position;
    /**
     * @brief Position at which homing is ended (before resetting position).
     *
     * This is only set when status is SUCCEEDED.  Together with start_position
     * it can be used to determine the distance the joint travelled during the
     * homing procedure (e.g. useful for home offset calibration).
     */
    double end_position;
};

/**
 * @brief The BlmcJointModule class is containing the joint information. It is
 * here to help converting the data from the motor side to the joint side. It
 * also allows the calibration of the joint position during initialization.
 */
class BlmcJointModule
{
public:
    /**
     * @brief Construct a new BlmcJointModule object
     *
     * @param motor is the C++ object allowing us to send commands and receive
     * sensor data.
     * @param motor_constant (\f$ k \f$) is the torque constant of the motor
     * \f$ \tau_{motor} = k * i_{motor} \f$
     * @param gear_ratio is the gear ratio between the motor and the joint.
     * @param zero_angle is the angle between the closest positive motor index
     * and the zero configuration.
     * @param reverse_polarity
     * @param max_current
     */
    BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle,
                    const bool& reverse_polarity = false,
                    const double& max_current = 2.1);

    /**
     * @brief Set the joint torque to be sent.
     *
     * @param desired_torque (Nm)
     */
    void set_torque(const double& desired_torque);

    /**
     * @brief Set the zero_angle. The zero_angle is the angle between the
     * closest positive motor index and the zero configuration.
     *
     * @param zero_angle (rad)
     */
    void set_zero_angle(const double& zero_angle);

    /**
     * @brief Define if the motor should turn clock-wize or counter clock-wize.
     *
     * @param reverse_polarity true:reverse rotation axis, false:do nothing.
     */
    void set_joint_polarity(const bool& reverse_polarity);

    /**
     * @brief send the joint torque to the motor. The conversion between joint
     * torque and motor current is done automatically.
     */
    void send_torque();

    /**
     * @brief Get the maximum admissible joint torque that can be applied.
     *
     * @return double
     */
    double get_max_torque() const;

    /**
     * @brief Get the sent joint torque.
     *
     * @return double (Nm).
     */
    double get_sent_torque() const;

    /**
     * @brief Get the measured joint torque.
     *
     * @return double (Nm).
     */
    double get_measured_torque() const;

    /**
     * @brief Get the measured angle of the joint.
     *
     * @return double (rad).
     */
    double get_measured_angle() const;

    /**
     * @brief Get the measured velocity of the joint. This data is computed on
     * board of the control card.
     *
     * @return double (rad/s).
     */
    double get_measured_velocity() const;

    /**
     * @brief Get the measured index angle. There is one index per motor
     * rotation so there are gear_ratio indexes per joint rotation.
     *
     * @return double (rad).
     */
    double get_measured_index_angle() const;

    /**
     * @brief Get the zero_angle_. These are the angle between the starting pose
     * and the theoretical zero pose.
     *
     * @return double (rad).
     */
    double get_zero_angle() const;

    /**
     * @brief Set control gains for PD position controller.
     *
     * @param kp P gain ( (Nm) / rad ).
     * @param kd D gain ( (Nm) / (rad/s) ).
     */
    void set_position_control_gains(double kp, double kd);

    /**
     * @brief Execute one iteration of the position controller.
     *
     * @param target_position_rad  Target position (rad).
     *
     * @return Torque command (Nm).
     */
    double execute_position_controller(double target_position_rad) const;

    /**
     * @deprecated !!!!!!!
     * @brief This method calibrate the joint position knowing the angle between
     * the closest (in positive torque) motor index and the theoretical zero
     * pose. Warning, this method should be called in a real time thread!
     *
     * @param[in][out] angle_zero_to_index (rad) this is the angle between the
     * closest (in positive torque) motor index and the theoretical zero pose.
     * @param[out] index_angle (rad) is the angle where we met the index. This
     * angle is relative to the configuration when the robot booted.
     * @param[in] mechanical_calibration defines if the leg started in the zero
     * configuration or not
     * @return true if success.
     * @return false if problem arose.
     */
    bool calibrate(double& angle_zero_to_index,
                   double& index_angle,
                   bool mechanical_calibration = false);

    /**
     * @brief Set zero position relative to current position
     *
     * @param home_offset_rad  Offset from home position to zero position.
     *     Unit: radian.
     */
    void homing_at_current_position(double home_offset_rad);

    /**
     * @brief Initialize the homing procedure.
     *
     * This has to be called before update_homing().
     *
     * @param joint_id ID of the joint.  This is only used for debug prints.
     * @param search_distance_limit_rad  Maximum distance the motor moves while
     *     searching for the encoder index.  Unit: radian.
     * @param home_offset_rad  Offset from home position to zero position.
     *     Unit: radian.
     * @param profile_step_size_rad  Distance by which the target position of
     * the position profile is changed in each step.  Set to a negative value to
     *     search for the next encoder index in negative direction.  Unit:
     *     radian.
     */
    void init_homing(int joint_id,
                     double search_distance_limit_rad,
                     double home_offset_rad,
                     double profile_step_size_rad = 0.001);

    /**
     * @brief Perform one step of homing on encoder index.
     *
     * Searches for the next encoder index in positive direction and, when
     * found, sets it as home position.
     *
     * Only performs one step, so this method needs to be called in a loop. This
     * method only set the control, one *MUST* send the control for the motor
     * after calling this method.
     *
     * The motor is moved with a position profile until either the encoder index
     * is reached or the search distance limit is exceeded.  The position is
     * controlled with a simple PD controller.
     *
     * If the encoder index is found, its position is used as home position.
     * The zero position is offset from the home position by adding the "home
     * offset" to it (i.e. zero = home pos. + home offset).
     * If the search distance limit is reached before the encoder index occurs,
     * the homing fails.
     *
     * @return Status of the homing procedure.
     */
    HomingReturnCode update_homing();

    /**
     * @brief Get distance between start and end position of homing.
     *
     * Compute the distance that the joint moved between initialization of
     * homing and reaching the home position.
     *
     * This can be used to determine the home offset by first moving the joint
     * to the desired zero position, then executing the homing and finally
     * calling this function which will provide the desired home offset.
     *
     * @return Distance between start and end position of homing.
     */
    double get_distance_travelled_during_homing() const;

private:
    /**
     * @brief Convert from joint torque to motor current.
     *
     * @param[in] torque is the input joint
     * @return double the equivalent motor current.
     */
    double joint_torque_to_motor_current(double torque) const;

    /**
     * @brief Convert from motor current to joint torque.
     *
     * @param current is the motor current.
     * @return double is the equivalent joint torque.
     */
    double motor_current_to_joint_torque(double current) const;

    /**
     * @brief Get motor measurements and check if there are data or not.
     *
     * @param measurement_id is the id of the measurement you want to get.
     * check: blmc_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    double get_motor_measurement(const mi& measurement_id) const;

    /**
     * @brief Get the last motor measurement index for a specific data. If there
     * was no data yet, return NaN
     *
     * @param measurement_id is the id of the measurement you want to get.
     * check: blmc_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    long int get_motor_measurement_index(const mi& measurement_id) const;

    /**
     * @brief This is the pointer to the motor interface.
     */
    std::shared_ptr<blmc_drivers::MotorInterface> motor_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    double motor_constant_;
    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor
     * rotation and the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    double gear_ratio_;
    /**
     * @brief This is the distance between the closest positive index and the
     * zero configuration.
     */
    double zero_angle_;
    /**
     * @brief This change the motor rotation direction.
     */
    double polarity_;
    /**
     * @brief This is the maximum current we can apply during one experiment.
     * The program shut down if this value is achieved.
     */
    double max_current_;

    //! @brief P gain of the position PD controller.
    double position_control_gain_p_;
    //! @brief D gain of the position PD controller.
    double position_control_gain_d_;

    struct HomingState homing_state_;
};

/**
 * @brief BlmcJointModule_ptr shortcut for the shared pointer BlmcJointModule
 * type
 */
typedef std::shared_ptr<BlmcJointModule> BlmcJointModule_ptr;

/**
 * @brief This class defines an interface to a collection of BLMC joints. It
 * creates a BLMCJointModule for every blmc_driver::MotorInterface provided.
 *
 * @tparam COUNT
 */
template <int COUNT>
class BlmcJointModules
{
public:
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface.
     */
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    /**
     * @brief Construct a new BlmcJointModules object.
     *
     * @param motors
     * @param motor_constants
     * @param gear_ratios
     * @param zero_angles
     */
    BlmcJointModules(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
            motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        set_motor_array(
            motors, motor_constants, gear_ratios, zero_angles, max_currents);
    }
    /**
     * @brief Construct a new BlmcJointModules object.
     */
    BlmcJointModules()
    {
    }
    /**
     * @brief Set the motor array, by creating the corresponding modules.
     *
     * @param motors
     * @param motor_constants
     * @param gear_ratios
     * @param zero_angles
     */
    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
            motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                                                            motor_constants[i],
                                                            gear_ratios[i],
                                                            zero_angles[i],
                                                            false,
                                                            max_currents[i]);
        }
    }
    /**
     * @brief Send the registered torques to all modules.
     */
    void send_torques()
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    /**
     * @brief Set the polarities of the joints
     * (see BlmcJointModule::set_joint_polarity)
     *
     * @param reverse_polarity
     */
    void set_joint_polarities(std::array<bool, COUNT> reverse_polarities)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_joint_polarity(reverse_polarities[i]);
        }
    }
    /**
     * @brief Register the joint torques to be sent for all modules.
     *
     * @param desired_torques (Nm)
     */
    void set_torques(const Vector& desired_torques)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    /**
     * @brief Get the maximum admissible joint torque that can be applied.
     *
     * @return Vector (N/m)
     */
    Vector get_max_torques()
    {
        Vector max_torques;
        for (size_t i = 0; i < COUNT; ++i)
        {
            max_torques[i] = modules_[i]->get_max_torque();
        }
        return max_torques;
    }

    /**
     * @brief Get the previously sent torques.
     *
     * @return Vector (Nm)
     */
    Vector get_sent_torques() const
    {
        Vector torques;

        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_sent_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint torques.
     *
     * @return Vector (Nm)
     */
    Vector get_measured_torques() const
    {
        Vector torques;

        for (size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_measured_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint angles.
     *
     * @return Vector (rad)
     */
    Vector get_measured_angles() const
    {
        Vector positions;

        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_measured_angle();
        }
        return positions;
    }

    /**
     * @brief Get the measured joint velocities.
     *
     * @return Vector (rad/s)
     */
    Vector get_measured_velocities() const
    {
        Vector velocities;

        for (size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_measured_velocity();
        }
        return velocities;
    }

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @param zero_angles (rad)
     */
    void set_zero_angles(const Vector& zero_angles)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_angles(i));
        }
    }
    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     *
     * @return Vector (rad)
     */
    Vector get_zero_angles() const
    {
        Vector positions;

        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_zero_angle();
        }
        return positions;
    }
    /**
     * @brief Get the index_angles. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     *
     * @return Vector (rad)
     */
    Vector get_measured_index_angles() const
    {
        Vector index_angles;

        for (size_t i = 0; i < COUNT; i++)
        {
            index_angles(i) = modules_[i]->get_measured_index_angle();
        }
        return index_angles;
    }

    /**
     * @brief Set position control gains for the specified joint.
     *
     * @param joint_id  ID of the joint (in range `[0, COUNT)`).
     * @param kp P gain.
     * @param kd D gain.
     */
    void set_position_control_gains(size_t joint_id, double kp, double kd)
    {
        modules_[joint_id]->set_position_control_gains(kp, kd);
    }

    /**
     * @brief Set position control gains for all joints.
     *
     * @param kp P gains.
     * @param kd D gains.
     */
    void set_position_control_gains(Vector kp, Vector kd)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            set_position_control_gains(i, kp[i], kd[i]);
        }
    }

    /**
     * @brief Perform homing for all joints at endstops.
     *
     * See BlmcJointModule::homing_at_current_position for description of the
     * arguments.
     *
     * @return Final status of the homing procedure (since homing happens at
     * current,position, procedure always returns success).
     */
    HomingReturnCode execute_homing_at_current_position(Vector home_offset_rad)
    {
        // Initialise homing for all joints
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->homing_at_current_position(home_offset_rad[i]);
        }

        return HomingReturnCode::SUCCEEDED;
    }

    /**
     * @brief Perform homing for all joints.
     *
     * If one of the joints fails, the complete homing fails.  Otherwise it
     * loops until all joints finished.
     * If a joint is finished while others are still running, it is held at the
     * home position.
     *
     * See BlmcJointModule::update_homing for details on the homing procedure.
     *
     * See BlmcJointModule::init_homing for description of the arguments.
     *
     * @return Final status of the homing procedure (either SUCCESS if all
     *     joints succeeded or the return code of the first joint that failed).
     */
    HomingReturnCode execute_homing(
        double search_distance_limit_rad,
        Vector home_offset_rad,
        Vector profile_step_size_rad = Vector::Constant(0.001))
    {
        // Initialise homing for all joints
        for (size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->init_homing((int)i,
                                     search_distance_limit_rad,
                                     home_offset_rad[i],
                                     profile_step_size_rad[i]);
        }

        // run homing for all joints until all of them are done
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);  // TODO magic number
        HomingReturnCode homing_status;
        do
        {
            bool all_succeeded = true;
            homing_status = HomingReturnCode::RUNNING;

            for (size_t i = 0; i < COUNT; i++)
            {
                HomingReturnCode joint_result = modules_[i]->update_homing();

                all_succeeded &= (joint_result == HomingReturnCode::SUCCEEDED);

                if (joint_result == HomingReturnCode::NOT_INITIALIZED ||
                    joint_result == HomingReturnCode::FAILED)
                {
                    homing_status = joint_result;
                    // abort homing
                    break;
                }
            }
            if (homing_status == HomingReturnCode::RUNNING)
            {
                for (unsigned i = 0; i < COUNT; ++i)
                {
                    modules_[i]->send_torque();
                }
            }

            if (all_succeeded)
            {
                homing_status = HomingReturnCode::SUCCEEDED;
            }

            spinner.spin();
        } while (homing_status == HomingReturnCode::RUNNING);

        return homing_status;
    }

    //! @see BlmcJointModule::get_distance_travelled_during_homing
    Vector get_distance_travelled_during_homing() const
    {
        Vector dist;
        for (unsigned i = 0; i < COUNT; ++i)
        {
            dist[i] = modules_[i]->get_distance_travelled_during_homing();
        }
        return dist;
    }

    /**
     * @brief Allow the robot to go to a desired pose. Once the control done
     * 0 torques is sent.
     *
     * @param angle_to_reach_rad (rad)
     * @param average_speed_rad_per_sec (rad/sec)
     * @return GoToReturnCode
     */
    GoToReturnCode go_to(Vector angle_to_reach_rad,
                         double average_speed_rad_per_sec = 1.0)
    {
        // Compute a min jerk trajectory
        Vector initial_joint_positions = get_measured_angles();
        double final_time = (angle_to_reach_rad - initial_joint_positions)
                                .array()
                                .abs()
                                .maxCoeff() /
                            average_speed_rad_per_sec;

        std::array<TimePolynome<5>, COUNT> min_jerk_trajs;
        for (unsigned i = 0; i < COUNT; i++)
        {
            min_jerk_trajs[i].set_parameters(final_time,
                                             initial_joint_positions[i],
                                             0.0 /*initial speed*/,
                                             angle_to_reach_rad[i]);
        }

        // run got_to for all joints
        real_time_tools::Spinner spinner;
        double sampling_period = 0.001;  // TODO magic number
        spinner.set_period(sampling_period);
        GoToReturnCode go_to_status;
        double current_time = 0.0;
        do
        {
            // TODO: add a security if error gets too big
            for (unsigned i = 0; i < COUNT; i++)
            {
                double desired_pose = min_jerk_trajs[i].compute(current_time);
                double desired_torque =
                    modules_[i]->execute_position_controller(desired_pose);
                modules_[i]->set_torque(desired_torque);
            }
            for (unsigned i = 0; i < COUNT; ++i)
            {
                modules_[i]->send_torque();
            }
            go_to_status = GoToReturnCode::RUNNING;

            current_time += sampling_period;
            spinner.spin();

        } while (current_time < (final_time + sampling_period));

        // Stop all motors (0 torques) after the destination achieved
        for (unsigned i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(0.0);
        }
        for (unsigned i = 0; i < COUNT; ++i)
        {
            modules_[i]->send_torque();
        }

        Vector final_pos = get_measured_angles();
        if ((angle_to_reach_rad - final_pos).isMuchSmallerThan(1.0, 1e-3))
        {
            go_to_status = GoToReturnCode::SUCCEEDED;
        }
        else
        {
            go_to_status = GoToReturnCode::FAILED;
        }
        return go_to_status;
    }

private:
    /**
     * @brief These are the BLMCJointModule objects corresponding to a robot.
     */
    std::array<std::shared_ptr<BlmcJointModule>, COUNT> modules_;
};

}  // namespace blmc_drivers
