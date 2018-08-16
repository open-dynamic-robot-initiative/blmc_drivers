#pragma once

#include <blmc_can/blmc_can.h>

#include <memory>
#include <string>

#include <utils/timer.hpp>
#include <utils/threadsafe_object.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <devices/motorboard.hpp>


class MotorInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;

    /// outputs ================================================================
    std::vector<std::string> measurement_names_ = {"current",
                                                   "position",
                                                   "velocity",
                                                   "encoder"};

    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> current_target() = 0;
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~MotorInterface() {}
};

class Motor: public MotorInterface
{
    std::map<std::string, std::string> motor_to_board_name_;
    std::shared_ptr<CanMotorboard> board_;

public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        return board_->measurement(motor_to_board_name_.at(name));
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> current_target()
    {
        return board_->control(motor_to_board_name_.at("current_target"));
    }

    virtual void send_if_input_changed()
    {
        board_->send_if_input_changed();
    }

    /// ========================================================================

    Motor(std::shared_ptr<CanMotorboard> board, bool motor_id):
        board_(board)
    {
        for(size_t i = 0; i < measurement_names_.size(); i++)
        {
            motor_to_board_name_[measurement_names_[i]] =
                    measurement_names_[i] + "_" + std::to_string(motor_id);
        }
        motor_to_board_name_["current_target"] =
                "current_target_" + std::to_string(motor_id);
    }
};


class MotorTemperature
{
public:
    MotorTemperature(double room_temperature):
        room_temperature_(room_temperature),
        temperature_(room_temperature)
    {
    }

    void update(double current, double delta_time)
    {
        temperature_ =
                room_temperature_ +
                exp(-0.003 * delta_time) * (temperature_ - room_temperature_) +
                0.03 * delta_time * pow(current, 2);
    }

    double get()
    {
        return temperature_;
    }

private:
    double room_temperature_;
    double temperature_;
};



class SafeMotor: public Motor
{
    MotorTemperature temperature_;

public:
    SafeMotor(std::shared_ptr<CanMotorboard> board, bool motor_id):
        Motor(board, motor_id), temperature_(30)
    {
        osi::start_thread(&SafeMotor::loop, this);
    }

private:
    static void
#ifndef __XENO__
    *
#endif
    loop(void* instance_pointer)
    {
        ((SafeMotor*)(instance_pointer))->loop();
    }

    void loop()
    {
        Timer<10> time_logger("controller");
        while(true)
        {
            Timer<>::sleep_ms(1);
            if(measurement("current")->history_length() == 0)
                continue;

            double current = measurement("current")->current_element();
            double target_current = current_target()->current_element();
            double velocity = measurement("velocity")->current_element();



            temperature_.update(current, 0.001);

            // print -----------------------------------------------------------
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 100) == 0)
            {
                osi::print_to_screen("--------------------------\n");
                osi::print_to_screen("current: %f\n", current);
                osi::print_to_screen("target current: %f\n", target_current);
                osi::print_to_screen("velocity: %f\n", velocity);


                osi::print_to_screen("temperature: %f\n", temperature_.get());
                osi::print_to_screen("--------------------------\n");

            }
        }
    }
};
