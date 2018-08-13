#include <devices/finger.hpp>
#include <devices/analogsensor.hpp>

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
            double analog_measurement = analog_sensor_->measurement()->current_element();
            double current_target = 2 * (analog_measurement - 0.5);

            finger_->control("current_target_interior")->append(current_target);
            finger_->control("current_target_center")->append(current_target);
            finger_->control("current_target_tip")->append(current_target);
            finger_->send_if_input_changed();

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
    std::shared_ptr<MotorInterface> motor_1 = std::make_shared<Motor>(board1, 0);
    std::shared_ptr<MotorInterface> motor_2 = std::make_shared<Motor>(board1, 1);
    std::shared_ptr<MotorInterface> motor_3 = std::make_shared<Motor>(board2, 0);

    std::shared_ptr<AnalogsensorInterface> analog_sensor_1 = std::make_shared<Analogsensor>(board1, 0);

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
