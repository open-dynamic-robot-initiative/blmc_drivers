#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;
typedef std::shared_ptr<blmc_drivers::AnalogSensor> Slider_ptr;
typedef std::pair<SafeMotor_ptr, Slider_ptr> PairMotorSlider;

class Controller
{
public:
  Controller(std::vector<PairMotorSlider> motor_slider_pairs):
    motor_slider_pairs_(motor_slider_pairs)
  {}

  void start_loop()
  {
    osi::start_thread(&Controller::loop, this);
  }

private:
  std::vector<PairMotorSlider> motor_slider_pairs_;

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
      for(std::vector<PairMotorSlider>::iterator pair_it = motor_slider_pairs_.begin() ;
          pair_it != motor_slider_pairs_.end() ; ++pair_it)
      {
        SafeMotor_ptr motor = pair_it->first;
        Slider_ptr slider = pair_it->second;

        analog_measurement = slider->get_measurement()->newest_element();
        desired_pose = (analog_measurement - 0.5);
        actual_pose = motor->get_measurement(
                        blmc_position_index)->newest_element();
        actual_velocity = motor->get_measurement(
                            blmc_velocity_index)->newest_element();

        desired_current = kp * (desired_pose - actual_pose) -
                          kd * (actual_velocity);

        if(desired_current > 1.0) {
            desired_current = 1.0;
        } else if (desired_current < -1.0) {
            desired_current = -1.0;
        }

        motor->set_current_target(desired_current);
        motor->send_if_input_changed();

        // print -----------------------------------------------------------
        Timer<>::sleep_ms(1);
        time_logger.end_and_start_interval();
        if ((time_logger.count() % 1000) == 0)
        {
            rt_printf("sending current: %f\n", desired_current);
            // time_logger.print_status();
        }
      }//endfor
    }//endwhile
  }// end mthod loop
}; // end class Controller definition




int main(int argc, char **argv)
{
  osi::initialize_realtime_printing();

  // create bus and boards -------------------------------------------------
#ifdef __XENO__
  auto can_bus0 = std::make_shared<blmc_drivers::CanBus>("rtcan0");
  auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("rtcan1");
  auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("rtcan2");
  auto can_bus3 = std::make_shared<blmc_drivers::CanBus>("rtcan3");
#else
  auto can_bus0 = std::make_shared<blmc_drivers::CanBus>("can0");
  auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can1");
  auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("can2");
  auto can_bus3 = std::make_shared<blmc_drivers::CanBus>("can3");
#endif
  auto board0 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus0);
  auto board1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
  auto board2 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);
  auto board3 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus3);

  // create motors and sensors ---------------------------------------------
  double max_current = 1;

  std::vector<PairMotorSlider> motor_slider_list;
  motor_slider_list.clear();

  // two individual motors on individual leg style mounting on the left of the
  // table
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board0, 0, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board0, 0)));
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board0, 1, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board0, 1)));

  // two individual motors with a wheel on top
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board1, 0, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board1, 0)));
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board1, 1, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board1, 1)));

  // the leg style mounting
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board2, 0, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board2, 0)));
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board2, 1, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board2, 1)));

  // the hopper style mounting
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board3, 0, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board3, 0)));
  motor_slider_list.push_back(PairMotorSlider(
        std::make_shared<blmc_drivers::SafeMotor>   (board3, 1, max_current),
        std::make_shared<blmc_drivers::AnalogSensor>(board3, 1)));

  rt_printf("motors and sliders are set up \n");

  Controller controller(motor_slider_list);

  rt_printf("controller is set up \n");

  controller.start_loop();

  rt_printf("control loop started \n");

  while(true)
  {
    Timer<>::sleep_ms(10);
  }

  return 0;
}
