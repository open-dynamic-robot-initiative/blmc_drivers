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
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;

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



//class SafeMotor: public Motor
//{
//public:
//    SafeMotor(std::shared_ptr<CanMotorboard> board, bool motor_id):
//        Motor(board, motor_id)
//    {
//        temperature_.set(StampedScalar(room_temperature_));
//        osi::start_thread(&SafeMotor::loop, this);
//    }


//    // send input data ---------------------------------------------------------
//    virtual void send_control(const StampedScalar& control,
//                              const std::string& name)
//    {
//    }

//private:
//    ThreadsafeObject<StampedScalar> temperature_;

//    const double room_temperature_ = 30;

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
//        Timer<10> time_logger("controller", 1000);
//        while(true)
//        {
//            wait_for_measurement("current");
//            StampedScalar current = get_measurement("current");




//            // print -----------------------------------------------------------
//            Timer<>::sleep_ms(1);
//            time_logger.end_and_start_interval();
//            if ((time_logger.count() % 1000) == 0)
//            {
//            }
//        }
//    }
//};
