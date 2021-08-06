/**
 * @file sine_position_control.hpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "blmc_drivers/devices/analog_sensor.hpp"
#include "blmc_drivers/devices/motor.hpp"

namespace blmc_drivers
{
/**
 * @brief This is a simple shortcut
 */
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;

/**
 * @brief This is a basic PD controller to be used in the demos of this package.
 */
class SinePositionControl
{
public:
    /**
     * @brief Construct a new SinePositionControl object.
     *
     * @param motor_slider_pairs
     */
    SinePositionControl(std::vector<SafeMotor_ptr> motor_list)
        : motor_list_(motor_list)
    {
        encoders_.clear();
        velocities_.clear();
        currents_.clear();
        control_buffer_.clear();

        for (size_t i = 0; i < motor_list.size(); ++i)
        {
            encoders_.push_back(std::deque<double>());
            currents_.push_back(std::deque<double>());
            velocities_.push_back(std::deque<double>());
            control_buffer_.push_back(std::deque<double>());
            encoders_.back().clear();
            velocities_.back().clear();
            currents_.back().clear();
            control_buffer_.back().clear();
        }
        stop_loop_ = false;
        kp_ = 0.0;
        kd_ = 0.0;
    }

    /**
     * @brief Destroy the SinePositionControl object
     */
    ~SinePositionControl()
    {
        stop_loop_ = true;
        rt_thread_.join();
    }

    /**
     * @brief This method is a helper to start the thread loop.
     */
    void start_loop()
    {
        rt_thread_.create_realtime_thread(&SinePositionControl::loop, this);
    }

    /**
     * @brief Stop the control and dump the data
     */
    void stop_loop();

    void set_gains(double kp, double kd)
    {
        kp_ = kp;
        kd_ = kd;
    }

private:
    /**
     * @brief This is list of motors
     */
    std::vector<SafeMotor_ptr> motor_list_;
    /**
     * @brief This is the real time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((SinePositionControl*)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop();

    /**
     * @brief managing the stopping of the loop
     */
    bool stop_loop_;

    /**
     * @brief memory_buffer_size_ is the max size of the memory buffer.
     */
    unsigned memory_buffer_size_;

    /**
     * @brief Encoder data
     */
    std::vector<std::deque<double> > encoders_;

    /**
     * @brief Velocity data
     */
    std::vector<std::deque<double> > velocities_;

    /**
     * @brief current data
     */
    std::vector<std::deque<double> > currents_;

    /**
     * @brief control_buffer_
     */
    std::vector<std::deque<double> > control_buffer_;

    /**
     * @brief Controller proportional gain.
     */
    double kp_;

    /**
     * @brief Controller derivative gain.
     */
    double kd_;

};  // end class SinePositionControl definition

}  // namespace blmc_drivers
