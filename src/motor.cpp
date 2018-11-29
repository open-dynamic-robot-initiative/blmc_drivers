/**
 * @file motor.cpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-27
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <blmc_drivers/devices/motor.hpp>

namespace blmc_drivers
{
  
Motor::Motor(Ptr<MotorBoardInterface> board, bool motor_id):
    board_(board),
    motor_id_(motor_id) 
{

}

Motor::Ptr<const Motor::ScalarTimeseries> Motor::get_measurement(
  const int& index) const
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

Motor::Ptr<const Motor::ScalarTimeseries> Motor::get_current_target() const
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

Motor::Ptr<const Motor::ScalarTimeseries> Motor::get_sent_current_target() const
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

void Motor::set_current_target(const double& current_target)
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

SafeMotor::SafeMotor(
  Motor::Ptr<MotorBoardInterface> board,
  bool motor_id,
  const double& max_current_target,
  const size_t& history_length):
      Motor(board, motor_id),
      max_current_target_(max_current_target)
{
    current_target_ = std::make_shared<ScalarTimeseries>(history_length);
}

void SafeMotor::set_current_target(const double& current_target)
{
    current_target_->append(current_target);

    // limit current to avoid overheating ----------------------------------
    double safe_current_target = std::min(current_target,
                                          max_current_target_);
    safe_current_target = std::max(safe_current_target,
                                    -max_current_target_);

//    // limit velocity to avoid breaking the robot --------------------------
//    if(get_measurement(velocity)->length() > 0 &&
//            std::fabs(get_measurement(velocity)->newest_element()) > 0.5)
//        safe_current_target = 0;

    Motor::set_current_target(safe_current_target);
}

} // namespace blmc_drivers
