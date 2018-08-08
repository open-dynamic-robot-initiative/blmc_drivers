#include <new_api.hpp>

class Controller
{
private:
std::shared_ptr<FingerInterface> finger_;
std::shared_ptr<AnalogsensorInterface> analog_sensor_;

public:
    Controller(std::shared_ptr<FingerInterface> finger,
               std::shared_ptr<AnalogsensorInterface> analog_sensor):
        finger_(finger), analog_sensor_(analog_sensor) { }

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

            StampedData<double> stamped_current_target(current_target, -1, -1);



            finger_->send_control(stamped_current_target, "current_target_interior");
            finger_->send_control(stamped_current_target, "current_target_center");
            finger_->send_control(stamped_current_target, "current_target_tip");



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
    auto can_bus1 = std::make_shared<XenomaiCanbus>("rtcan0");
    auto can_bus2 = std::make_shared<XenomaiCanbus>("rtcan1");
    auto board1 = std::make_shared<CanMotorboard>(can_bus1);
    auto board2 = std::make_shared<CanMotorboard>(can_bus2);

    // create motors and sensors ---------------------------------------------
    std::shared_ptr<MotorInterface> motor_1 = std::make_shared<Motor>(board1, BLMC_MTR1);
    std::shared_ptr<MotorInterface> motor_2 = std::make_shared<Motor>(board1, BLMC_MTR2);
    std::shared_ptr<MotorInterface> motor_3 = std::make_shared<Motor>(board2, BLMC_MTR1);

    std::shared_ptr<AnalogsensorInterface> analog_sensor_1 = std::make_shared<Analogsensor>(board1, BLMC_ADC_A);

    std::shared_ptr<FingerInterface> finger = std::make_shared<Finger>(motor_1, motor_2, motor_3);


    Controller controller1(finger, analog_sensor_1);


    // somehow this is necessary to be able to use some of the functionality
    osi::make_this_thread_realtime();
    board1->enable();
    board2->enable();

    controller1.start_loop();


    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
