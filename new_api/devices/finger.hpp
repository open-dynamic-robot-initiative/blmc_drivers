#pragma once

#include <memory>
#include <string>
#include <map>


#include <utils/threadsafe_object.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <devices/motor.hpp>


class FingerInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;

    /// outputs ================================================================
    std::vector<std::string> measurement_names_ = {"current_interior",
                                                   "current_center",
                                                   "current_tip",
                                                   "position_interior",
                                                   "position_center",
                                                   "position_tip"
                                                   "velocity_interior",
                                                   "velocity_center",
                                                   "velocity_tip",
                                                   "encoder_interior",
                                                   "encoder_center",
                                                   "encoder_tip"};

    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;

    /// inputs =================================================================
    std::vector<std::string> control_names_ = {"current_target_interior",
                                               "current_target_center",
                                               "current_target_tip"};

    virtual std::shared_ptr<ScalarTimeseries> control(std::string name) = 0;
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~FingerInterface() {}
};



//class Finger: public FingerInterface
//{
//    std::map<std::string, std::shared_ptr<MotorInterface>> motors_;

//public:
//    /// outputs ================================================================
//    virtual std::shared_ptr<const ScalarTimeseries>
//    measurement(std::string name) const
//    {
//        std::string motor_name, content_name;
//        parse_name(name, motor_name, content_name);

//        return motors_.at(motor_name)->measurement(content_name);
//    }

//    /// inputs =================================================================
//    virtual std::shared_ptr<ScalarTimeseries> control(std::string name)
//    {
//        std::string motor_name, content_name;
//        parse_name(name, motor_name, content_name);

//        return motors_.at(motor_name)->current_target();
//    }

//    virtual void send_if_input_changed()
//    {
//        motors_["interior"]->send_if_input_changed();
//        motors_["center"]->send_if_input_changed();
//        motors_["tip"]->send_if_input_changed();
//    }

//    /// ========================================================================

//    Finger(std::shared_ptr<MotorInterface> interior_motor,
//           std::shared_ptr<MotorInterface> center_motor,
//           std::shared_ptr<MotorInterface> tip_motor)
//    {
//        motors_["interior"] = interior_motor;
//        motors_["center"] = center_motor;
//        motors_["tip"] = tip_motor;
//    }

//private:
//    void parse_name(const std::string& name,
//                    std::string& motor_name, std::string& content_name) const
//    {
//        size_t _position = name.rfind("_");
//        content_name = name.substr(0, _position);
//        motor_name = name;
//        motor_name.erase(0, _position + 1);
//    }
//};


