#include <blmc_drivers/devices/motor.hpp>
#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/devices/leg.hpp>
#include <math.h>

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
                // time_logger.print_status();
            }
        }
    }
};

class LegController
{
private:
    std::shared_ptr<blmc_drivers::Leg> leg_;
    std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor_;

public:
    LegController(std::shared_ptr<blmc_drivers::Leg> leg,
               std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor):
        leg_(leg), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        osi::start_thread(&LegController::loop, this);
    }

private:
    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((LegController*)(instance_pointer))->loop();
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
        Timer<10> time_logger("Leg controller");
        while(true)
        {
            double analog_measurement =
                    analog_sensor_->get_measurement()->newest_element();
            double position_target = (analog_measurement - 0.5);

            double position_hip = leg_->get_motor_measurement(blmc_drivers::Leg::hip,
                                                         blmc_drivers::Leg::position)->newest_element();
            double velocity_hip = leg_->get_motor_measurement(blmc_drivers::Leg::hip,
                                                         blmc_drivers::Leg::velocity)->newest_element();

            double position_knee = leg_->get_motor_measurement(blmc_drivers::Leg::knee,
                                                         blmc_drivers::Leg::position)->newest_element();
            double velocity_knee = leg_->get_motor_measurement(blmc_drivers::Leg::knee,
                                                         blmc_drivers::Leg::velocity)->newest_element();

            double kp = 5;
            double kd = 1;
            double current_target_knee = kp*(position_target - position_knee) -
                                         kd*(velocity_knee);
            double current_target_hip = kp*(position_target - position_hip) -
                                         kd*(velocity_hip);

            if(current_target_knee > 1.0) {
                current_target_knee = 1.0;
            } else if (current_target_knee < -1.0) {
                current_target_knee = -1.0;
            }

            if(current_target_hip > 1.0) {
                current_target_hip = 1.0;
            } else if (current_target_hip < -1.0) {
                current_target_hip = -1.0;
            }

            leg_->set_current_target(current_target_knee, blmc_drivers::Leg::knee);
            leg_->set_current_target(current_target_hip, blmc_drivers::Leg::hip);
            leg_->send_if_input_changed();

            // print -----------------------------------------------------------
            Timer<>::sleep_ms(1);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                rt_printf("sending current: %f\n", current_target_knee);
                // time_logger.print_status();
            }
        }
    }
};




int main(int argc, char **argv)
{  
    osi::initialize_realtime_printing();

    // create bus and boards -------------------------------------------------

    // this is the id of the cans bus (plug behind the computer). see https://atlas.is.localnet/confluence/pages/viewpage.action?pageId=44958260
    // use netstat -i repeatedly to see wich can bus value is changing. (Later labels behind the computer identify it).
    auto can_bus = std::make_shared<blmc_drivers::CanBus>("can3");

    // the board is used to communicate with the can bus, it takes one upon creation
    auto board = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus);
    
    // create motors and sensors ---------------------------------------------
    auto motor_hip = std::make_shared<blmc_drivers::Motor>(board, 0);
    auto motor_knee = std::make_shared<blmc_drivers::Motor>(board, 1); 

    auto leg = std::make_shared<blmc_drivers::Leg>(motor_hip, motor_knee);
    rt_printf("leg is set up \n");

    // on the board there is an analog input (slider) at slot 0 (of the analog input)
    auto analog_sensor = std::make_shared<blmc_drivers::AnalogSensor>(board, 0);
    // on the board there is an analog input (slider) at slot 0 (of the analog input)
    //auto analog_sensor = std::make_shared<blmc_drivers::AnalogSensor>(board, 1);
    rt_printf("sensors are set up \n");

    LegController leg_controller(leg, analog_sensor);
    rt_printf("controllers are set up \n");

    leg_controller.start_loop();
    rt_printf("loops have started \n");

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
