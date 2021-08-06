/**
 * @file demo_leg.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <math.h>
#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/devices/leg.hpp>
#include <blmc_drivers/devices/motor.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

#include <signal.h>
#include <atomic>

#include <pd_control.hpp>

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int)
{
    StopDemos = true;
}

/**
 * @brief This is a simple PD control on one motor and one slider
 */
class Controller
{
private:
    /**
     * @brief one motor.
     */
    std::shared_ptr<blmc_drivers::Motor> motor_;
    /**
     * @brief one slider.
     */
    std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor_;
    /**
     * @brief the realt time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

public:
    /**
     * @brief Construct a new Controller object.
     *
     * @param motor
     * @param analog_sensor
     */
    Controller(std::shared_ptr<blmc_drivers::Motor> motor,
               std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor)
        : motor_(motor), analog_sensor_(analog_sensor)
    {
    }
    /**
     * @brief main control loop
     */
    void start_loop()
    {
        rt_thread_.create_realtime_thread(&Controller::loop, this);
    }

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((Controller*)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

private:
    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop()
    {
        real_time_tools::Spinner time_spinner;
        time_spinner.set_period(0.001);  // 1kz loop
        size_t count = 0;
        while (true)
        {
            double analog_measurement =
                analog_sensor_->get_measurement()->newest_element();
            double current_target = 4 * (analog_measurement - 0.5);

            motor_->set_current_target(current_target);
            motor_->send_if_input_changed();

            // print -----------------------------------------------------------
            time_spinner.spin();
            if ((count % 1000) == 0)
            {
                rt_printf("sending current: %f\n", current_target);
                // time_logger.print_status();
            }
            count++;
        }
    }
};

/**
 * @brief Simple PD control on the leg
 */
class LegController
{
private:
    /**
     * @brief is the leg to control
     */
    std::shared_ptr<blmc_drivers::Leg> leg_;
    /**
     * @brief is the list of sliders
     */
    std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor_;
    /**
     * @brief is the real time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief manages the shutdown of the controller
     */
    bool stop_loop_;

public:
    /**
     * @brief Construct a new LegController object
     *
     * @param leg
     * @param analog_sensor
     */
    LegController(std::shared_ptr<blmc_drivers::Leg> leg,
                  std::shared_ptr<blmc_drivers::AnalogSensor> analog_sensor)
        : leg_(leg), analog_sensor_(analog_sensor)
    {
        stop_loop_ = false;
    }

    /**
     * @brief Destroy the LegController object
     */
    ~LegController()
    {
        stop_loop_ = true;
        rt_thread_.join();
    }

    /**
     * @brief helper to strat the real time thread.
     */
    void start_loop()
    {
        rt_thread_.create_realtime_thread(&Controller::loop, this);
    }

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((LegController*)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

private:
    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop()
    {
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);  // 1kz loop
        size_t count = 0;
        while (!stop_loop_)
        {
            double analog_measurement =
                analog_sensor_->get_measurement()->newest_element();
            double position_target = (analog_measurement - 0.5);

            double position_hip =
                leg_->get_motor_measurement(blmc_drivers::Leg::hip,
                                            blmc_drivers::Leg::position)
                    ->newest_element();
            double velocity_hip =
                leg_->get_motor_measurement(blmc_drivers::Leg::hip,
                                            blmc_drivers::Leg::velocity)
                    ->newest_element();

            double position_knee =
                leg_->get_motor_measurement(blmc_drivers::Leg::knee,
                                            blmc_drivers::Leg::position)
                    ->newest_element();
            double velocity_knee =
                leg_->get_motor_measurement(blmc_drivers::Leg::knee,
                                            blmc_drivers::Leg::velocity)
                    ->newest_element();

            double kp = 5;
            double kd = 1;
            double current_target_knee =
                kp * (position_target - position_knee) - kd * (velocity_knee);
            double current_target_hip =
                kp * (position_target - position_hip) - kd * (velocity_hip);

            if (current_target_knee > 1.0)
            {
                current_target_knee = 1.0;
            }
            else if (current_target_knee < -1.0)
            {
                current_target_knee = -1.0;
            }

            if (current_target_hip > 1.0)
            {
                current_target_hip = 1.0;
            }
            else if (current_target_hip < -1.0)
            {
                current_target_hip = -1.0;
            }

            leg_->set_current_target(current_target_knee,
                                     blmc_drivers::Leg::knee);
            leg_->set_current_target(current_target_hip,
                                     blmc_drivers::Leg::hip);
            leg_->send_if_input_changed();

            // print -----------------------------------------------------------
            spinner.spin();
            if ((count % 1000) == 0)
            {
                rt_printf("sending current: %f\n", current_target_knee);
                // time_logger.print_status();
            }
            count++;
        }
    }
};

int main(int, char**)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;

    // create bus and boards -------------------------------------------------

    // this is the id of the cans bus (plug behind the computer).
    // see
    // https://atlas.is.localnet/confluence/pages/viewpage.action?pageId=44958260
    // use netstat -i repeatedly to see wich can bus value is changing.
    // normally labels behind the computer identify it.
    auto can_bus = std::make_shared<blmc_drivers::CanBus>("can3");

    // the board is used to communicate with the can bus, it takes one upon
    // creation
    auto board = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus);

    // create motors and sensors ---------------------------------------------
    auto motor_hip = std::make_shared<blmc_drivers::Motor>(board, 0);
    auto motor_knee = std::make_shared<blmc_drivers::Motor>(board, 1);

    auto leg = std::make_shared<blmc_drivers::Leg>(motor_hip, motor_knee);
    rt_printf("leg is set up \n");

    // on the board there is an analog input (slider) at slot 0 (of the analog
    // input)
    auto analog_sensor = std::make_shared<blmc_drivers::AnalogSensor>(board, 0);
    // on the board there is an analog input (slider) at slot 0 (of the analog
    // input)
    // auto analog_sensor = std::make_shared<blmc_drivers::AnalogSensor>(board,
    // 1);
    rt_printf("sensors are set up \n");

    LegController leg_controller(leg, analog_sensor);
    rt_printf("controllers are set up \n");

    leg_controller.start_loop();
    rt_printf("loops have started \n");

    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.001);
    }

    return 0;
}
