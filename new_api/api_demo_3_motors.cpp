#include <new_api.hpp>

class Controller
{
private:
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<Analogsensor> analog_sensor_;

public:
    Controller(std::shared_ptr<Motor> motor,
               std::shared_ptr<Analogsensor> analog_sensor):
        motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        osi::start_thread(&Controller::loop, this);
    }

    static void
#ifndef __XENO__
    *
#endif
    loop(void* instance_pointer)
    {
        ((Controller*)(instance_pointer))->loop();
    }

    void loop()
    {
        Timer<10> time_logger("controller", 1000);
        while(true)
        {
            double current_target =
                    2 * (analog_sensor_->get_measurement("analog").get_data() - 0.5);
            motor_->send_control(StampedData<double>(current_target, -1, -1), "current_target");

            // print -----------------------------------------------------------
            Timer<>::sleep_ms(1);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                osi::print_to_screen("sending current: %f\n", current_target);
            }
        }
    }
};




int main(int argc, char **argv)
{  
    osi::initialize_realtime_printing();

    // create bus and boards -------------------------------------------------

#ifdef __XENO__
    auto can_bus1 = std::make_shared<XenomaiCanbus>("rtcan0");
    auto can_bus2 = std::make_shared<XenomaiCanbus>("rtcan1");
#else
    auto can_bus1 = std::make_shared<XenomaiCanbus>("can0");
    auto can_bus2 = std::make_shared<XenomaiCanbus>("can1");
#endif
    auto board1 = std::make_shared<CanMotorboard>(can_bus1);
    auto board2 = std::make_shared<CanMotorboard>(can_bus2);

    // create motors and sensors ---------------------------------------------
    auto motor_1 = std::make_shared<Motor>(board1, BLMC_MTR1);
    auto motor_2 = std::make_shared<Motor>(board1, BLMC_MTR2);
    auto motor_3 = std::make_shared<Motor>(board2, BLMC_MTR1);

    auto analog_sensor_1 = std::make_shared<Analogsensor>(board1, BLMC_ADC_A);
    auto analog_sensor_2 = std::make_shared<Analogsensor>(board1, BLMC_ADC_B);
    auto analog_sensor_3 = std::make_shared<Analogsensor>(board2, BLMC_ADC_A);

    Controller controller1(motor_1, analog_sensor_1);
    Controller controller2(motor_2, analog_sensor_2);
    Controller controller3(motor_3, analog_sensor_3);

    // somehow this is necessary to be able to use some of the functionality
    osi::make_this_thread_realtime();
    board1->enable();
    board2->enable();

    controller1.start_loop();
    controller2.start_loop();
    controller3.start_loop();

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
