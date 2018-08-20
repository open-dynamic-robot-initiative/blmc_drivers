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

    enum Measurement {current, position, velocity, encoder, measurement_count};

    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(const size_t& index = 0) const = 0;

    /// inputs =================================================================
    virtual std::shared_ptr<const ScalarTimeseries> control() const = 0;

    /// log ====================================================================
    virtual std::shared_ptr<const ScalarTimeseries> sent_control() const = 0;

    /// ========================================================================
    virtual void set_control(const double& control) = 0;
    virtual void set_command(const MotorboardCommand& command) = 0;

    virtual void send_if_input_changed() = 0;

    virtual ~MotorInterface() {}
};

class Motor: public MotorInterface
{
protected:
    std::shared_ptr<MotorboardInterface> board_;
    bool motor_id_;

public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(const size_t& index = 0) const
    {
        if(motor_id_ == 0)
        {
            switch(index)
            {
            case current: return board_->measurement(MotorboardInterface::current_0);
            case position: return board_->measurement(MotorboardInterface::position_0);
            case velocity: return board_->measurement(MotorboardInterface::velocity_0);
            case encoder: return board_->measurement(MotorboardInterface::encoder_0);
            }
        }
        else
        {
            switch(index)
            {
            case current: return board_->measurement(MotorboardInterface::current_1);
            case position: return board_->measurement(MotorboardInterface::position_1);
            case velocity: return board_->measurement(MotorboardInterface::velocity_1);
            case encoder: return board_->measurement(MotorboardInterface::encoder_1);
            }
        }
    }

    /// inputs =================================================================
    virtual std::shared_ptr<const ScalarTimeseries> control() const
    {
        if(motor_id_ == 0)
        {
            return board_->control(MotorboardInterface::current_target_0);
        }
        else
        {
            return board_->control(MotorboardInterface::current_target_1);
        }
    }

    /// log ====================================================================
    virtual std::shared_ptr<const ScalarTimeseries> sent_control() const
    {
        if(motor_id_ == 0)
        {
           return board_->sent_control(MotorboardInterface::current_target_0);
        }
        else
        {
            return board_->sent_control(MotorboardInterface::current_target_1);
        }
    }

    /// ========================================================================
    virtual void set_control(const double& control)
    {
        if(motor_id_ == 0)
        {
            board_->set_control(control, MotorboardInterface::current_target_0);
        }
        else
        {
            board_->set_control(control, MotorboardInterface::current_target_1);
        }
    }

    virtual void set_command(const MotorboardCommand& command)
    {
        board_->set_command(command);
    }

    virtual void send_if_input_changed()
    {
        board_->send_if_input_changed();
    }


    Motor(std::shared_ptr<MotorboardInterface> board, bool motor_id):
        board_(board),
        motor_id_(motor_id) { }

    virtual ~Motor() { }
};

/// \todo: the velocity limit should be implemented in a smoother way,
/// and the parameters should be passed in the constructor
class SafeMotor: public Motor
{
    double max_control_ = 2.0;
public:
    std::shared_ptr<ScalarTimeseries> control_;

    virtual void set_control(const double& control)
    {
        control_->append(control);

        // limit current to avoid overheating ----------------------------------
        double safe_control = std::min(control, max_control_);
        safe_control = std::max(safe_control, -max_control_);

        // limit velocity to avoid breaking the robot --------------------------
        if(measurement(velocity)->history_length() > 0 &&
                std::fabs(measurement(velocity)->current_element()) > 0.5)
            safe_control = 0;

        Motor::set_control(safe_control);
    }

    virtual std::shared_ptr<const ScalarTimeseries> control() const
    {
        return control_;
    }

    SafeMotor(std::shared_ptr<MotorboardInterface> board, bool motor_id):
        Motor(board, motor_id)
    {
        control_ = std::make_shared<ScalarTimeseries>(1000);
    }
};

//class MotorTemperature
//{
//public:
//    MotorTemperature(double room_temperature):
//        room_temperature_(room_temperature),
//        temperature_(room_temperature)
//    {
//    }

//    void update(double current, double delta_time)
//    {
//        temperature_ =
//                room_temperature_ +
//                exp(-0.003 * delta_time) * (temperature_ - room_temperature_) +
//                0.03 * delta_time * pow(current, 2);
//    }

//    double get()
//    {
//        return temperature_;
//    }

//private:
//    double room_temperature_;
//    double temperature_;
//};



//class SafeMotor: public Motor
//{
//    MotorTemperature temperature_;

//public:
//    SafeMotor(std::shared_ptr<MotorboardInterface> board, bool motor_id):
//        Motor(board, motor_id), temperature_(30)
//    {
//        osi::start_thread(&SafeMotor::loop, this);
//    }

//private:
//    static void
//#ifndef __XENO__
//    *
//#endif
//    loop(void* instance_pointer)
//    {
//        ((SafeMotor*)(instance_pointer))->loop();
//    }

//    void loop()
//    {
//        Timer<10> time_logger("controller");
//        while(true)
//        {
//            Timer<>::sleep_ms(1);
//            if(measurement("current")->history_length() == 0)
//                continue;

//            double current = measurement("current")->current_element();
//            double target_current = current_target()->current_element();
//            double velocity = measurement("velocity")->current_element();



//            temperature_.update(current, 0.001);

//            // print -----------------------------------------------------------
//            time_logger.end_and_start_interval();
//            if ((time_logger.count() % 100) == 0)
//            {
//                osi::print_to_screen("--------------------------\n");
//                osi::print_to_screen("current: %f\n", current);
//                osi::print_to_screen("target current: %f\n", target_current);
//                osi::print_to_screen("velocity: %f\n", velocity);


//                osi::print_to_screen("temperature: %f\n", temperature_.get());
//                osi::print_to_screen("--------------------------\n");

//            }
//        }
//    }
//};
