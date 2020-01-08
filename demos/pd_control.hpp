/**
 * @file pd_control.hpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck Gesellschaft, License BSD-3-Clause
 */

#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/devices/analog_sensor.hpp"

namespace blmc_drivers{

/**
 * @brief This is a simple shortcut
 */
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;
/**
 * @brief This is a simple shortcut
 */
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;
/**
 * @brief This is a simple shortcut
 */
typedef std::pair<SafeMotor_ptr, Slider_ptr> PairMotorSlider;

/**
 * @brief This is a basic PD controller to be used in the demos of this package.
 */
class PDController
{
public:
  /**
   * @brief Construct a new PDController object.
   * 
   * @param motor_slider_pairs 
   */
  PDController(std::vector<PairMotorSlider> motor_slider_pairs):
    motor_slider_pairs_(motor_slider_pairs)
  {
    stop_loop=false;
  }

  /**
   * @brief Destroy the PDController object
   */
  ~PDController()
  {
    stop_loop=true;
    rt_thread_.join();
  }

  /**
   * @brief This method is a helper to start the thread loop.
   */
  void start_loop()
  {
    rt_thread_.create_realtime_thread(&PDController::loop, this);
  }

private:

  /**
   * @brief This is a pair of motor and sliders so that we associate one with
   * the other.
   */
  std::vector<PairMotorSlider> motor_slider_pairs_;
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
    ((PDController*)(instance_pointer))->loop();
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
  bool stop_loop;

}; // end class PDController definition

}// namespace blmc_drivers
