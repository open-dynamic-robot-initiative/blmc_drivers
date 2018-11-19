#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

class Controller
{
private:
    std::shared_ptr<blmc_drivers::Motor> motor_;
    std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor_;

public:
    Controller(std::shared_ptr<blmc_drivers::Motor> motor,
               std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor):
        motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        osi::start_thread(&Controller::loop, this);
    }

private:
    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((Controller*)(instance_pointer))->loop();
    }


    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop()
    {
      const int & blmc_position_index = blmc_drivers::MotorInterface::MeasurementIndex::position;
      const int & blmc_velocity_index = blmc_drivers::MotorInterface::MeasurementIndex::velocity;
      double analog_measurement = 0.0;
      double desired_pose = 0.0;
      double actual_pose = 0.0;
      double actual_velocity = 0.0;
      double kp = 5;
      double kd = 1;
      // here is the control in current (Amper)
      double desired_current = 0.0;
      Timer<10> time_logger("controller");
      while(true)
      {
          analog_measurement = analog_sensor_->get_measurement()->newest_element();
          desired_pose = 3.0 * (analog_measurement - 0.5);
          actual_pose = motor_->get_measurement(
                          blmc_position_index)->newest_element();
          actual_velocity = motor_->get_measurement(
                              blmc_velocity_index)->newest_element();

          desired_current = kp * (desired_pose - actual_pose) -
                            kd * (actual_velocity);

          if(desired_current > 1.0) {
              desired_current = 1.0;
          } else if (desired_current < -1.0) {
              desired_current = -1.0;
          }

          motor_->set_current_target(desired_current);
          motor_->send_if_input_changed();

          // print -----------------------------------------------------------
          Timer<>::sleep_ms(1);
          time_logger.end_and_start_interval();
          if ((time_logger.count() % 1000) == 0)
          {
              rt_printf("sending current: %f\n", desired_current);
          }
      }
    }
};




int main(int argc, char **argv)
{
    osi::initialize_realtime_printing();

    // create bus and boards -------------------------------------------------
#ifdef __XENO__
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("rtcan0");
    auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("rtcan1");
#else
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can3");
#endif
    auto board1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);

    // create motors and sensors ---------------------------------------------
    auto motor_1 = std::make_shared<blmc_drivers::SafeMotor>(board1, 0);
    auto motor_2 = std::make_shared<blmc_drivers::SafeMotor>(board1, 1);

    rt_printf("motors are set up \n");

    auto analog_sensor_1 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 0);
    auto analog_sensor_2 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 1);

    rt_printf("sensors are set up \n");

    Controller controller1(motor_1, analog_sensor_1);
    Controller controller2(motor_2, analog_sensor_2);

    rt_printf("controllers are set up \n");

    controller1.start_loop();
    controller2.start_loop();

    rt_printf("loops have started \n");

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}

