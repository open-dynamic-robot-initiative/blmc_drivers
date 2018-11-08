#pragma once

#include <memory>
#include <string>

#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/utils/threadsafe_object.hpp>
#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <blmc_drivers/devices/motor_board.hpp>
#include <blmc_drivers/devices/device_interface.hpp>


namespace blmc_drivers
{

class MotorInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    template<typename Type> using Ptr = std::shared_ptr<Type>;


    enum MeasurementIndex {current, position, velocity, encoder_index,
                           measurement_count};

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_measurement(
            const int& index = 0) const = 0;

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_current_target() const = 0;
    virtual Ptr<const ScalarTimeseries> get_sent_current_target() const = 0;

    /// setters ================================================================
    virtual void set_current_target(const double& current_target) = 0;
    virtual void set_command(const MotorBoardCommand& command) = 0;

    /// sender =================================================================
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~MotorInterface() {}
};

class Motor: public MotorInterface
{
protected:
    Ptr<MotorBoardInterface> board_;
    bool motor_id_;

public:
    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_measurement(const int& index = 0) const
    {
        if(motor_id_ == 0)
        {
            switch(index)
            {
            case current:
                return board_->get_measurement(MotorBoardInterface::current_0);
            case position:
                return board_->get_measurement(MotorBoardInterface::position_0);
            case velocity:
                return board_->get_measurement(MotorBoardInterface::velocity_0);
            case encoder_index:
                return board_->get_measurement(
                            MotorBoardInterface::encoder_index_0);
            }
        }
        else
        {
            switch(index)
            {
            case current:
                return board_->get_measurement(MotorBoardInterface::current_1);
            case position:
                return board_->get_measurement(MotorBoardInterface::position_1);
            case velocity:
                return board_->get_measurement(MotorBoardInterface::velocity_1);
            case encoder_index:
                return board_->get_measurement(
                            MotorBoardInterface::encoder_index_1);
            }
        }
    }

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_current_target() const
    {
        if(motor_id_ == 0)
        {
            return board_->get_control(MotorBoardInterface::current_target_0);
        }
        else
        {
            return board_->get_control(MotorBoardInterface::current_target_1);
        }
    }
    virtual Ptr<const ScalarTimeseries> get_sent_current_target() const
    {
        if(motor_id_ == 0)
        {
           return board_->get_sent_control(
                       MotorBoardInterface::current_target_0);
        }
        else
        {
            return board_->get_sent_control(
                        MotorBoardInterface::current_target_1);
        }
    }

    /// setters ================================================================
    virtual void set_current_target(const double& current_target)
    {
        if(motor_id_ == 0)
        {
            board_->set_control(current_target,
                                MotorBoardInterface::current_target_0);
        }
        else
        {
            board_->set_control(current_target,
                                MotorBoardInterface::current_target_1);
        }
    }
    virtual void set_command(const MotorBoardCommand& command)
    {
        board_->set_command(command);
    }

    /// sender =================================================================
    virtual void send_if_input_changed()
    {
        board_->send_if_input_changed();
    }

    /// ========================================================================


    Motor(Ptr<MotorBoardInterface> board, bool motor_id):
        board_(board),
        motor_id_(motor_id) { }

    virtual ~Motor() { }
};





/// \todo: the velocity limit should be implemented in a smoother way,
/// and the parameters should be passed in the constructor
class SafeMotor: public Motor
{
private:
    double max_current_target_;
    Ptr<ScalarTimeseries> current_target_;

public:
    virtual void set_current_target(const double& current_target)
    {
        current_target_->append(current_target);

        // limit current to avoid overheating ----------------------------------
        double safe_current_target = std::min(current_target,
                                              max_current_target_);
        safe_current_target = std::max(safe_current_target,
                                       -max_current_target_);

//        // limit velocity to avoid breaking the robot --------------------------
//        if(get_measurement(velocity)->length() > 0 &&
//                std::fabs(get_measurement(velocity)->newest_element()) > 0.5)
//            safe_current_target = 0;

        Motor::set_current_target(safe_current_target);
    }

    virtual Ptr<const ScalarTimeseries> current_target() const
    {
        return current_target_;
    }

    SafeMotor(Ptr<MotorBoardInterface> board, bool motor_id,
              const double& max_current_target = 2.0,
              const size_t& history_length = 1000):
        Motor(board, motor_id),
        max_current_target_(max_current_target)
    {
        current_target_ = std::make_shared<ScalarTimeseries>(history_length);
    }
};

}

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
//    SafeMotor(Ptr<MotorBoardInterface> board, bool motor_id):
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
//            if(measurement("current")->length() == 0)
//                continue;

//            double current = measurement("current")->newest_element();
//            double target_current = current_target()->newest_element();
//            double velocity = measurement("velocity")->newest_element();



//            temperature_.update(current, 0.001);

//            // print -----------------------------------------------------------
//            time_logger.end_and_start_interval();
//            if ((time_logger.count() % 100) == 0)
//            {
//                rt_printf("--------------------------\n");
//                rt_printf("current: %f\n", current);
//                rt_printf("target current: %f\n", target_current);
//                rt_printf("velocity: %f\n", velocity);


//                rt_printf("temperature: %f\n", temperature_.get());
//                rt_printf("--------------------------\n");

//            }
//        }
//    }
//};
