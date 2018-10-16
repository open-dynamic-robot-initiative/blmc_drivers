//#include <devices/motor.hpp>

//class Controller
//{
//private:
//    std::shared_ptr<Motor> motor_;

//public:
//    Controller(std::shared_ptr<Motor> motor): motor_(motor) { }

//    void start_loop()
//    {
//        osi::start_thread(&Controller::loop, this);
//    }

//    static void
//#ifndef __XENO__
//    *
//#endif
//    loop(void* instance_pointer)
//    {
//        ((Controller*)(instance_pointer))->loop();
//    }

//    void loop()
//    {
//        Timer<10> time_logger("controller", 1000);
//        while(true)
//        {
//            double current_target = 0.5;
////            motor_->send_control(StampedData<double>(current_target, -1, -1), "current_target");

//            motor_->current_target()->append(current_target);
//            motor_->send_if_input_changed();


//            // print -----------------------------------------------------------
//            Timer<>::sleep_ms(1);
//            time_logger.end_and_start_interval();
//            if ((time_logger.count() % 1000) == 0)
//            {
//                osi::realtime_printf("sending current: %f\n", current_target);
//            }
//        }
//    }
//};




int main(int argc, char **argv)
{
//    osi::initialize_realtime_printing();

//    // create bus and boards -------------------------------------------------
//    auto can_bus1 = std::make_shared<CanBus>("rtcan0");
//    auto board1 = std::make_shared<CanBusMotorBoard>(can_bus1);

//    // create motors and sensors ---------------------------------------------
//    auto motor_1 = std::make_shared<Motor>(board1, BLMC_MTR1);

//    Controller controller1(motor_1);

//    // somehow this is necessary to be able to use some of the functionality
//    osi::make_this_thread_realtime();
//    board1->enable();
//    controller1.start_loop();

//    while(true)
//    {
//        Timer<>::sleep_ms(10);
//    }

    return 0;
}
