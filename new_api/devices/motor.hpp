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

    enum Measurement {current, position, velocity, encoder_index, measurement_count};

    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(const size_t& index = 0) const = 0;

    /// inputs =================================================================
    virtual std::shared_ptr<const ScalarTimeseries> current_target() const = 0;

    /// log ====================================================================
    virtual std::shared_ptr<const ScalarTimeseries> sent_current_target() const = 0;

    /// ========================================================================
    virtual void set_current_target(const double& current_target) = 0;
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
            case current:
                return board_->measurement(MotorboardInterface::current_0);
            case position:
                return board_->measurement(MotorboardInterface::position_0);
            case velocity:
                return board_->measurement(MotorboardInterface::velocity_0);
            case encoder_index:
                return board_->measurement(MotorboardInterface::encoder_index_0);
            }
        }
        else
        {
            switch(index)
            {
            case current:
                return board_->measurement(MotorboardInterface::current_1);
            case position:
                return board_->measurement(MotorboardInterface::position_1);
            case velocity:
                return board_->measurement(MotorboardInterface::velocity_1);
            case encoder_index:
                return board_->measurement(MotorboardInterface::encoder_index_1);
            }
        }
    }

    /// inputs =================================================================
    virtual std::shared_ptr<const ScalarTimeseries> current_target() const
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
    virtual std::shared_ptr<const ScalarTimeseries> sent_current_target() const
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
    virtual void set_current_target(const double& current_target)
    {
        if(motor_id_ == 0)
        {
            board_->set_control(current_target,
                                MotorboardInterface::current_target_0);
        }
        else
        {
            board_->set_control(current_target,
                                MotorboardInterface::current_target_1);
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
    double max_current_target_ = 2.0;
public:
    std::shared_ptr<ScalarTimeseries> current_target_;

    virtual void set_current_target(const double& current_target)
    {
        current_target_->append(current_target);

        // limit current to avoid overheating ----------------------------------
        double safe_current_target = std::min(current_target,
                                              max_current_target_);
        safe_current_target = std::max(safe_current_target,
                                       -max_current_target_);

        // limit velocity to avoid breaking the robot --------------------------
        if(measurement(velocity)->history_length() > 0 &&
                std::fabs(measurement(velocity)->newest_element()) > 0.5)
            safe_current_target = 0;

        Motor::set_current_target(safe_current_target);
    }

    virtual std::shared_ptr<const ScalarTimeseries> current_target() const
    {
        return current_target_;
    }

    SafeMotor(std::shared_ptr<MotorboardInterface> board, bool motor_id):
        Motor(board, motor_id)
    {
        current_target_ = std::make_shared<ScalarTimeseries>(1000);
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
//        Timer<10> time_logger("current_targetler");
//        while(true)
//        {
//            Timer<>::sleep_ms(1);
//            if(measurement("current")->history_length() == 0)
//                continue;

//            double current = measurement("current")->newest_element();
//            double target_current = current_target()->newest_element();
//            double velocity = measurement("velocity")->newest_element();



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
