/**
 * @file motor_board.cpp
 * @author Felix Widmaier (felix.widmaier@tuebingen.mpg.de)
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file implements the classes from
 * "blmc_drivers/devices/motor_board.hpp"
 * @version 0.1
 * @date 2018-11-26
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "real_time_tools/timer.hpp"
#include "blmc_drivers/devices/ethernet_wifi_motor_board.hpp"


namespace rt = real_time_tools;


namespace blmc_drivers
{

EthernetWifiMotorBoard::EthernetWifiMotorBoard(
    std::shared_ptr<MasterBoardInterface> master_board_interface,
    const size_t slave_id, const size_t history_length): MotorBoardInterface()
{
  // Save the ser input
  master_board_interface_ = master_board_interface;
  slave_id_ = slave_id;
  history_length_ = history_length;
  
  // get the udriver from its index:
  assert(N_SLAVES > slave_id_ && "The slave_id is not valid.");
  udriver_ = &(master_board_interface_->motor_drivers[slave_id_]);

 	// Initialisation of the slave (udriver), send the init commands
	udriver_->motor1->SetCurrentReference(0);
	udriver_->motor2->SetCurrentReference(0);
	udriver_->motor1->Enable();
	udriver_->motor2->Enable();
	udriver_->EnablePositionRolloverError();
	udriver_->SetTimeout(5);
	udriver_->Enable();

  // initialize the buffers
  measurement_ = create_vector_of_pointers<ScalarTimeseries>(measurement_count,
                                                             history_length);
  status_ = std::make_shared<StatusTimeseries>(history_length);
  control_ = create_vector_of_pointers<ScalarTimeseries>(control_count,
                                                         history_length);
  command_ = std::make_shared<CommandTimeseries>(history_length);
  sent_control_ = create_vector_of_pointers<ScalarTimeseries>(control_count,
                                                              history_length);
  sent_command_ = std::make_shared<CommandTimeseries>(history_length);

  // start the communication loop
  is_loop_active_ = true;
  rt_thread_.create_realtime_thread(&EthernetWifiMotorBoard::loop, this);
}

EthernetWifiMotorBoard::~EthernetWifiMotorBoard()
{
    is_loop_active_ = false;
    rt_thread_.join();
    // Initialisation of the slave (udriver), send the init commands
    udriver_->motor1->SetCurrentReference(0);
    udriver_->motor2->SetCurrentReference(0);
    udriver_->motor1->Disable();
    udriver_->motor2->Disable();
    udriver_->DisablePositionRolloverError();
    udriver_->Disable();
}

/**
 * Output and status
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
EthernetWifiMotorBoard::get_measurement(const int& index) const
{
    return measurement_[index];
}

std::shared_ptr<const MotorBoardInterface::StatusTimeseries>
EthernetWifiMotorBoard::get_status() const
{
    return status_;
}

/**
 * input logs
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
EthernetWifiMotorBoard::get_control(const int& index) const
{
    return control_[index];
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
EthernetWifiMotorBoard::get_command() const
{
    return command_;
}

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
EthernetWifiMotorBoard::get_sent_control(const int& index) const
{
    return control_[index];
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
EthernetWifiMotorBoard::get_sent_command() const
{
    return sent_command_;
}

/**
 * Setters
 */

void EthernetWifiMotorBoard::set_control(
    const double& control, const int& index)
{
    rt_printf("EthernetWifiMotorBoard::set_control(): called.\n");
    control_[index]->append(control);
}
    
void EthernetWifiMotorBoard::set_command(const MotorBoardCommand& command)
{
    command_->append(command);
}

void EthernetWifiMotorBoard::send_if_input_changed()
{
    // send command if a new one has been set ----------------------------------
    if(command_->has_changed_since_tag())
    {
        switch (command_->newest_element().id_)
        {
        case MotorBoardCommand::ENABLE_SYS:
            if(command_->newest_element().content_ == MotorBoardCommand::ENABLE)
            {
                udriver_->Enable();
            }else
            {
                udriver_->Disable();
            }
            break;
        case MotorBoardCommand::ENABLE_MTR1:
            if(command_->newest_element().content_ == MotorBoardCommand::ENABLE)
            {
                udriver_->motor1->Enable();
            }else
            {
                udriver_->motor1->Disable();
            }
            break;
        case MotorBoardCommand::ENABLE_MTR2:
            if(command_->newest_element().content_ == MotorBoardCommand::ENABLE)
            {
                udriver_->motor2->Enable();
            }else
            {
                udriver_->motor2->Disable();
            }
            break;
        case MotorBoardCommand::ENABLE_VSPRING1:
            break;
        case MotorBoardCommand::ENABLE_VSPRING2:
            break;
        case MotorBoardCommand::SEND_CURRENT:
            break;
        case MotorBoardCommand::SEND_POSITION:
            break;
        case MotorBoardCommand::SEND_VELOCITY:
            break;
        case MotorBoardCommand::SEND_ADC6:
            break;
        case MotorBoardCommand::SEND_ENC_INDEX:
            break;
        case MotorBoardCommand::SEND_ALL:
            break;
        case MotorBoardCommand::SET_CAN_RECV_TIMEOUT:
            break;
        case MotorBoardCommand::ENABLE_POS_ROLLOVER_ERROR:
            if(command_->newest_element().content_ == MotorBoardCommand::ENABLE)
            {
                udriver_->
                    EnablePositionRolloverError();
            }else
            {
                udriver_->
                    DisablePositionRolloverError();
            }
            break;        
        default:
          throw std::runtime_error("EthernetWifiMotorBoard::send_if_input_changed:"
                                   " Error cannot send unkown command");
          break;
        }
    }

    // send controls -----------------------------------------------------------
    std::array<double, control_count> controls;
    assert(controls.size() == control_.size() && "Wrong control vector size");
    for(size_t i = 0; i < control_.size(); ++i)
    {
        if(control_[i]->length() == 0)
        {
            rt_printf("you tried to send control but no control has been set\n");
            exit(-1);
        }
        Index timeindex = control_[i]->newest_timeindex();
        controls[i] = (*control_[i])[timeindex];
        control_[i]->tag(timeindex);

        sent_control_[i]->append(controls[i]);
    }
    udriver_->motor1->SetCurrentReference(controls[0]);
    udriver_->motor2->SetCurrentReference(controls[1]);

}

bool EthernetWifiMotorBoard::is_ready()
{
    // check if the motors and if the udriver are enabled and ready.
    bool is_hardware_ready = true;
    is_hardware_ready =
        is_hardware_ready &&
        udriver_->motor1->IsEnabled() &&
        udriver_->motor2->IsEnabled() &&
        udriver_->motor1->IsReady() &&
        udriver_->motor2->IsReady();
    return is_hardware_ready;
}

void EthernetWifiMotorBoard::loop()
{
    // receive data from board in a loop ---------------------------------------
    rt::Spinner spinner;
    spinner.set_period(0.001); // period of 1ms
    while(is_loop_active_)
    {
        // This will read the last incomming packet and update all sensor fields.
        master_board_interface_->ParseSensorData();
        if(is_ready())
        {
            measurement_[current_0]->append(udriver_->motor1->GetCurrent());
            measurement_[current_1]->append(udriver_->motor2->GetCurrent());
            measurement_[position_0]->append(udriver_->motor1->GetPosition() /* * 2 * M_PI */);
            measurement_[position_1]->append(udriver_->motor2->GetPosition() /* * 2 * M_PI */);
            measurement_[velocity_0]->append(udriver_->motor1->GetVelocity() /* * 2 * M_PI * (1000./60.) */);
            measurement_[velocity_1]->append(udriver_->motor2->GetVelocity() /* * 2 * M_PI * (1000./60.) */);
            measurement_[analog_0]->append(0.0 /* udriver_->analogue1->getData() */);// not implemented yet in the API
            measurement_[analog_1]->append(0.0 /* udriver_->analogue2->getData() */);// not implemented yet in the API
            // here the interpretation of the message is different,
            // we get a motor index and a measurement
            if(udriver_->motor1->GetIndexToggleBit())
            {
              measurement_[encoder_index_0]->append(udriver_->motor1->GetPosition() /* * 2 * M_PI */);
            }
            if(udriver_->motor2->GetIndexToggleBit())
            {
              measurement_[encoder_index_0]->append(udriver_->motor2->GetPosition() /* * 2 * M_PI */);
            }
            MotorBoardStatus status;
            status.system_enabled = udriver_->is_enabled;
            status.motor1_enabled = udriver_->motor1->is_enabled;
            status.motor1_ready   = udriver_->motor1->is_ready;
            status.motor2_enabled = udriver_->motor2->is_enabled;
            status.motor2_ready   = udriver_->motor2->is_ready;
            status.error_code     = udriver_->error_code;
            status_->append(status);
            spinner.spin();
        }
    }
}

} // namespace blmc_drivers
