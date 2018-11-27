#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>

class Controller
{
private:
    std::shared_ptr<blmc_drivers::Motor> motor_;
    std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor_;
    real_time_tools::RealTimeThread rt_thread_;

public:
    Controller(std::shared_ptr<blmc_drivers::Motor> motor,
               std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor):
        motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        real_time_tools::create_realtime_thread(
          rt_thread_, &Controller::loop, this);
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
        Timer<10> time_logger("controller");
        while(true)
        {
            double analog_measurement =
                    analog_sensor_->get_measurement()->newest_element();
            double current_target = 4 * (analog_measurement - 0.5);

            motor_->set_current_target(current_target);
            motor_->send_if_input_changed();

            // print -----------------------------------------------------------
            Timer<>::sleep_ms(1);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                rt_printf("sending current: %f\n", current_target);
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
    auto can_bus1 = std::make_shared<blmc_drivers::CanBus>("can0");
    auto can_bus2 = std::make_shared<blmc_drivers::CanBus>("can1");
#endif
    auto board1 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
    auto board2 = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);

    // create motors and sensors ---------------------------------------------
    auto motor_1 = std::make_shared<blmc_drivers::SafeMotor>(board1, 0);
    auto motor_2 = std::make_shared<blmc_drivers::SafeMotor>(board1, 1);
    auto motor_3 = std::make_shared<blmc_drivers::SafeMotor>(board2, 0);

    rt_printf("motors are set up \n");

    auto analog_sensor_1 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 0);
    auto analog_sensor_2 = std::make_shared<blmc_drivers::AnalogSensor>(board1, 1);
    auto analog_sensor_3 = std::make_shared<blmc_drivers::AnalogSensor>(board2, 0);

    rt_printf("sensors are set up \n");

    Controller controller1(motor_1, analog_sensor_1);
    Controller controller2(motor_2, analog_sensor_2);
    Controller controller3(motor_3, analog_sensor_3);

    rt_printf("controllers are set up \n");

    controller1.start_loop();
    controller2.start_loop();
    controller3.start_loop();

    rt_printf("loops have started \n");

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
