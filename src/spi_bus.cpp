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
#include "blmc_drivers/devices/motor.hpp"
#include "blmc_drivers/devices/spi_bus.hpp"


namespace rt = real_time_tools;

namespace blmc_drivers
{

SpiBus::SpiBus(
    std::shared_ptr<MasterBoardInterface> master_board_interface,
    const size_t& nb_udrivers, const size_t& history_length): DeviceInterface()
{
  // Save the ser input
  main_board_interface_ = master_board_interface;
  nb_udrivers_ = nb_udrivers;
  history_length_ = history_length;
  
  // get the udriver from its index:
  assert(N_SLAVES >= nb_udrivers_ && "The number of controlled udriver is not valid.");

  // initialize the buffers
  measurement_ = create_vector_of_pointers<MotorInterface::ScalarTimeseries>(
      nb_udrivers_ * MotorBoardInterface::MeasurementIndex::measurement_count,
      history_length_);

  status_ = create_vector_of_pointers<MotorBoardInterface::StatusTimeseries>(
      nb_udrivers_,
      history_length_);

  control_ = create_vector_of_pointers<MotorInterface::ScalarTimeseries>(
      nb_udrivers_ * MotorBoardInterface::ControlIndex::control_count,
      history_length);

  command_ = create_vector_of_pointers<MotorBoardInterface::CommandTimeseries>(
      nb_udrivers_, history_length);

  sent_control_ = create_vector_of_pointers<MotorInterface::ScalarTimeseries>(
      nb_udrivers_ * MotorBoardInterface::ControlIndex::control_count,
      history_length);

  sent_command_ = create_vector_of_pointers<MotorBoardInterface::CommandTimeseries>(
      nb_udrivers_, history_length);

  // start the communication loop
  motors_are_paused_ = true;
  is_loop_active_ = true;
  rt_thread_.create_realtime_thread(&SpiBus::loop, this);
}

SpiBus::~SpiBus()
{
    is_loop_active_ = false;
    rt_thread_.join();
    // de-activate the robot
    for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
    {
        // Some aliases
        MotorDriver& udriver = main_board_interface_->motor_drivers[i];
        udriver.motor1->SetCurrentReference(0);
        udriver.motor2->SetCurrentReference(0);
        udriver.motor1->Disable();
        udriver.motor2->Disable();
        udriver.DisablePositionRolloverError();
        udriver.Disable();
    }
}

/**
 * Output and status
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
SpiBus::get_measurement(
    const size_t udriver_id,
    const MotorBoardInterface::MeasurementIndex& index) const
{
    return measurement_[udriver_id * 2 + index];
}

std::shared_ptr<const MotorBoardInterface::StatusTimeseries>
SpiBus::get_status(const size_t udriver_id) const
{
    return status_[udriver_id];
}

/**
 * input logs
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
SpiBus::get_control(const size_t udriver_id,
    const MotorBoardInterface::ControlIndex& index) const
{
    return control_[udriver_id * 2 + index];
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
SpiBus::get_command(const size_t udriver_id) const
{
    return command_[udriver_id];
}

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
SpiBus::get_sent_control(const size_t udriver_id,
    const MotorBoardInterface::ControlIndex& index) const
{
    return control_[udriver_id * 2 + index];
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
SpiBus::get_sent_command(const size_t udriver_id) const
{
    return sent_command_[udriver_id];
}

/**
 * Setters
 */

void SpiBus::set_control(const size_t udriver_id, const double& control,
                         const MotorBoardInterface::ControlIndex& index)
{
    control_[udriver_id * MotorBoardInterface::ControlIndex::control_count +
             index]->append(control);
}
    
void SpiBus::set_command(const size_t udriver_id, 
                         const MotorBoardCommand& command)
{
    command_[udriver_id]->append(command);
}

void SpiBus::send_if_input_changed()
{
    // send command if a new one has been set ----------------------------------
    bool commands_have_changed = false;
    for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
    {
        if(command_[i]->has_changed_since_tag())
        {
            commands_have_changed = true;
            break;
        }
    }
    if(commands_have_changed)
    {
        send_newest_command();
    }
    // send controls if a new one has been set ---------------------------------
    bool controls_have_changed = false;
    for(size_t i = 0 ; i < control_.size() ; ++i)
    {
        if(control_[i]->has_changed_since_tag())
        {
            controls_have_changed = true;
            break;
        }
    }
    if(controls_have_changed)
    {
        send_newest_controls();
    }
}

void SpiBus::send_newest_command()
{
    for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
    {
        // Some aliases
        MotorDriver& udriver = main_board_interface_->motor_drivers[i];
        const uint32_t& command_id = command_[i]->newest_element().id_;
        const int32_t& command_content = command_[i]->newest_element().content_;
        switch (command_id)
        {
            case MotorBoardCommand::ENABLE_SYS:
                command_content == MotorBoardCommand::ENABLE ?
                    udriver.Enable() : udriver.Disable();
                break;
            case MotorBoardCommand::ENABLE_MTR1:
                command_content == MotorBoardCommand::ENABLE ?
                    udriver.motor1->Enable() : udriver.motor1->Disable();
                break;
            case MotorBoardCommand::ENABLE_MTR2:
                command_content == MotorBoardCommand::ENABLE ?
                    udriver.motor2->Enable() : udriver.motor2->Disable();
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
                command_content == MotorBoardCommand::ENABLE ?
                    udriver.EnablePositionRolloverError() :
                    udriver.DisablePositionRolloverError();
                break;        
            default:
                throw std::runtime_error("SpiBus::send_if_input_changed:"
                                        " Error cannot send unkown command");
                break;
        } // end switch
    } // end for
}

void SpiBus::send_newest_controls()
{
    // send controls -----------------------------------------------------------
    for(size_t i = 0; i < control_.size(); ++i)
    {
        if(control_[i]->length() == 0)
        {
            rt_printf("You tried to send control on the motor[%ld] "
                      "but no control has been set\n", i);
            exit(-1);
        }
        MotorInterface::ScalarTimeseries::Index timeindex =
            control_[i]->newest_timeindex();

        double control = (*control_[i])[timeindex];
        
        main_board_interface_->motors[i].SetCurrentReference(control);
        control_[i]->tag(timeindex);
        sent_control_[i]->append(control);
    }
    motors_are_paused_ = false;
    main_board_interface_->SendCommand();
}

bool SpiBus::is_ready()
{
    // check if the motors and if the udriver are enabled and ready.
    bool is_hardware_ready = true;
    for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
    {
        MotorDriver& udriver = main_board_interface_->motor_drivers[i];
        is_hardware_ready = is_hardware_ready &&
                            udriver.motor1->IsEnabled() &&
                            udriver.motor2->IsEnabled() &&
                            udriver.motor1->IsReady() &&
                            udriver.motor2->IsReady();
    }
    return is_hardware_ready;
}

void SpiBus::loop()
{
    motors_are_paused_ = true;

    for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
    {
        MotorDriver& udriver = main_board_interface_->motor_drivers[i];
        // Initialisation of the udriver, send the init commands
        udriver.motor1->SetCurrentReference(0);
        udriver.motor2->SetCurrentReference(0);
        udriver.motor1->Enable();
        udriver.motor2->Enable();
        udriver.EnablePositionRolloverError();
        udriver.SetTimeout(5);
        udriver.Enable();
    }

    // some aliases
    size_t measurement_count = MotorBoardInterface::MeasurementIndex::measurement_count;
    size_t current_0 = MotorBoardInterface::MeasurementIndex::current_0;
    size_t current_1 = MotorBoardInterface::MeasurementIndex::current_1;
    size_t position_0 = MotorBoardInterface::MeasurementIndex::position_0;
    size_t position_1 = MotorBoardInterface::MeasurementIndex::position_1;
    size_t velocity_0 = MotorBoardInterface::MeasurementIndex::velocity_0;
    size_t velocity_1 = MotorBoardInterface::MeasurementIndex::velocity_1;
    size_t analog_0 = MotorBoardInterface::MeasurementIndex::analog_0;
    size_t analog_1 = MotorBoardInterface::MeasurementIndex::analog_1;
    size_t encoder_index_0 = MotorBoardInterface::MeasurementIndex::encoder_index_0;
    size_t encoder_index_1 = MotorBoardInterface::MeasurementIndex::encoder_index_1;
    // receive data from board in a loop ---------------------------------------
    rt::Spinner spinner;
    spinner.set_period(0.001); // period of 1ms
    while(is_loop_active_)
    {
        // This will read the last incomming packet and update all sensor fields.
        main_board_interface_->ParseSensorData();
    
        for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
        {
            MotorDriver& udriver = main_board_interface_->motor_drivers[i];
            measurement_[i* measurement_count + current_0]->append(udriver.motor1->GetCurrent());
            measurement_[i* measurement_count + current_1]->append(udriver.motor2->GetCurrent());
            measurement_[i* measurement_count + position_0]->append(udriver.motor1->GetPosition() /* * 2 * M_PI */);
            measurement_[i* measurement_count + position_1]->append(udriver.motor2->GetPosition() /* * 2 * M_PI */);
            measurement_[i* measurement_count + velocity_0]->append(udriver.motor1->GetVelocity() /* * 2 * M_PI * (1000./60.) */);
            measurement_[i* measurement_count + velocity_1]->append(udriver.motor2->GetVelocity() /* * 2 * M_PI * (1000./60.) */);
            measurement_[i* measurement_count + analog_0]->append(udriver.adc[0]);// not implemented yet in the API
            measurement_[i* measurement_count + analog_1]->append(udriver.adc[1]);// not implemented yet in the API
            // here the interpretation of the message is different,
            // we get a motor index and a measurement
            if(udriver.motor1->GetIndexToggleBit())
            {
                measurement_[encoder_index_0]->append(udriver.motor1->GetPosition() /* * 2 * M_PI */);
            }
            if(udriver.motor2->GetIndexToggleBit())
            {
                measurement_[encoder_index_1]->append(udriver.motor2->GetPosition() /* * 2 * M_PI */);
            }
            MotorBoardStatus status;
            status.system_enabled = udriver.is_enabled;
            status.motor1_enabled = udriver.motor1->is_enabled;
            status.motor1_ready   = udriver.motor1->is_ready;
            status.motor2_enabled = udriver.motor2->is_enabled;
            status.motor2_ready   = udriver.motor2->is_ready;
            status.error_code     = udriver.error_code;
            status_[i]->append(status);
        }
    
        if(!is_ready() && motors_are_paused_)
        {
            for(size_t i = 0 ; i < nb_udrivers_ ; ++i)
            {
                MotorDriver& udriver = main_board_interface_->motor_drivers[i];
                udriver.motor1->SetCurrentReference(0.0);
                udriver.motor2->SetCurrentReference(0.0);
            }
            main_board_interface_->SendCommand();
        }
        spinner.spin();
    }
}

void SpiBus::wait_is_ready()
{
    while(!is_ready())
    {
        rt::Timer::sleep_sec(0.001);
    }
}

} // namespace blmc_drivers
