#include <demos/finger.hpp>
#include <devices/analog_sensor.hpp>

class Controller
{
private:
std::shared_ptr<FingerInterface> finger_;
std::shared_ptr<AnalogSensorInterface> analog_sensor_;

public:
    Controller(std::shared_ptr<FingerInterface> finger,
               std::shared_ptr<AnalogSensorInterface> analog_sensor):
        finger_(finger), analog_sensor_(analog_sensor) { }

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
     * each motor of the finger robot.
     */
    void loop()
    {
        Timer<10> time_logger("controller", 1000);
        while(true)
        {
            double analog_measurement =
                    analog_sensor_->get_measurement()->newest_element();
            double current_target = 2 * (analog_measurement - 0.5);

            finger_->set_current_target(current_target,
                                        FingerInterface::interior);
            finger_->set_current_target(current_target,
                                        FingerInterface::center);
            finger_->set_current_target(current_target,
                                        FingerInterface::tip);
            finger_->send_if_input_changed();

            // print -----------------------------------------------------------
            Timer<>::sleep_ms(1);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                osi::realtime_printf("sending current: %f\n", current_target);
            }
        }
    }
};


int main(int argc, char **argv)
{
    osi::initialize_realtime_printing();

    // create bus and boards -------------------------------------------------
#ifdef __XENO__
    auto can_bus1 = std::make_shared<CanBus>("rtcan0");
    auto can_bus2 = std::make_shared<CanBus>("rtcan1");
#else
    auto can_bus1 = std::make_shared<CanBus>("can0");
    auto can_bus2 = std::make_shared<CanBus>("can1");
#endif
    auto board1 = std::make_shared<CanBusMotorBoard>(can_bus1);
    auto board2 = std::make_shared<CanBusMotorBoard>(can_bus2);

    // create motors and sensors ---------------------------------------------
    auto motor_1 = std::make_shared<SafeMotor>(board1, 0);
    auto motor_2 = std::make_shared<SafeMotor>(board1, 1);
    auto motor_3 = std::make_shared<SafeMotor>(board2, 0);
    auto analog_sensor = std::make_shared<AnalogSensor>(board1, 0);

    // create finger -----------------------------------------------------------
    auto finger = std::make_shared<Finger>(motor_1, motor_2, motor_3);

    // create controller and start control loop --------------------------------
    Controller controller(finger, analog_sensor);
    controller.start_loop();

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
