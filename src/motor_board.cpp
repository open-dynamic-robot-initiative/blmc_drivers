/**
 * @file motor_board.cpp
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

#include <blmc_drivers/devices/motor_board.hpp>

namespace blmc_drivers
{

CanBusMotorBoard::CanBusMotorBoard(
  std::shared_ptr<CanBusInterface> can_bus,
  const size_t& history_length):
    can_bus_(can_bus)
{
    measurement_  = create_vector_of_pointers<ScalarTimeseries>(
                measurement_count,
                history_length);
    status_       = std::make_shared<StatusTimeseries>(history_length);
    control_      = create_vector_of_pointers<ScalarTimeseries>(
                control_count,
                history_length);
    command_      = std::make_shared<CommandTimeseries>(history_length);
    sent_control_ = create_vector_of_pointers<ScalarTimeseries>(
                control_count,
                history_length);
    sent_command_ = std::make_shared<CommandTimeseries>(history_length);

    // initialize outputs --------------------------------------------------
    for(size_t i = 0; i < control_.size(); i++)
        control_[i]->append(0);

    is_loop_active_ = true;
    real_time_tools::create_realtime_thread(
          rt_thread_, &CanBusMotorBoard::loop, this);
}

CanBusMotorBoard::~CanBusMotorBoard()
{
    is_loop_active_ = false;
    real_time_tools::join_thread(rt_thread_);
    append_and_send_command(
                MotorBoardCommand(MotorBoardCommand::IDs::ENABLE_SYS,
                                  MotorBoardCommand::Contents::DISABLE));
}

void CanBusMotorBoard::send_if_input_changed()
{
    // initialize outputs --------------------------------------------------
    bool controls_have_changed = false;

    for(auto control : control_)
    {
        if(control->has_changed_since_tag())
            controls_have_changed = true;
    }
    if(controls_have_changed)
    {
        std::array<double, 2> controls_to_send;
        for(size_t i = 0; i < control_.size(); i++)
        {
            Index timeindex_to_send = control_[i]->newest_timeindex();
            controls_to_send[i] = (*control_[i])[timeindex_to_send];
            control_[i]->tag(timeindex_to_send);

            sent_control_[i]->append(controls_to_send[i]);
        }
        send_controls(controls_to_send);
    }

    if(command_->has_changed_since_tag())
    {
        Index timeindex_to_send = command_->newest_timeindex();
        MotorBoardCommand command_to_send = (*command_)[timeindex_to_send];
        command_->tag(timeindex_to_send);
        sent_command_->append(command_to_send);

        send_command(command_to_send);
    }
}

void CanBusMotorBoard::enable()
{
    append_and_send_command(MotorBoardCommand(
                                MotorBoardCommand::IDs::ENABLE_SYS,
                                MotorBoardCommand::Contents::ENABLE));
    append_and_send_command(MotorBoardCommand(
                                MotorBoardCommand::IDs::SEND_ALL,
                                MotorBoardCommand::Contents::ENABLE));
    append_and_send_command(MotorBoardCommand(
                                MotorBoardCommand::IDs::ENABLE_MTR1,
                                MotorBoardCommand::Contents::ENABLE));
    append_and_send_command(MotorBoardCommand(
                                MotorBoardCommand::IDs::ENABLE_MTR2,
                                MotorBoardCommand::Contents::ENABLE));
    append_and_send_command(MotorBoardCommand(
                                MotorBoardCommand::IDs::SET_CAN_RECV_TIMEOUT,
                                100));
}

void CanBusMotorBoard::send_controls(std::array<double, 2> controls)
{
    float current_mtr1 = controls[0];
    float current_mtr2 = controls[1];

    uint8_t data[8];
    uint32_t q_current1, q_current2;

    // Convert floats to Q24 values
    q_current1 = float_to_q24(current_mtr1);
    q_current2 = float_to_q24(current_mtr2);

    // Motor 1
    data[0] = (q_current1 >> 24) & 0xFF;
    data[1] = (q_current1 >> 16) & 0xFF;
    data[2] = (q_current1 >> 8) & 0xFF;
    data[3] =  q_current1 & 0xFF;

    // Motor 2
    data[4] = (q_current2 >> 24) & 0xFF;
    data[5] = (q_current2 >> 16) & 0xFF;
    data[6] = (q_current2 >> 8) & 0xFF;
    data[7] =  q_current2 & 0xFF;

    CanBusFrame can_frame;
    can_frame.id = CanframeIDs::IqRef;
    for(size_t i = 0; i < 7; i++)
    {
        can_frame.data[i] = data[i];
    }
    can_frame.dlc = 8;

    can_bus_->set_input_frame(can_frame);
    can_bus_->send_if_input_changed();
}

void CanBusMotorBoard::send_command(MotorBoardCommand command)
{
    uint32_t id = command.id_;
    int32_t content = command.content_;

    uint8_t data[8];

    // content
    data[0] = (content >> 24) & 0xFF;
    data[1] = (content >> 16) & 0xFF;
    data[2] = (content >> 8) & 0xFF;
    data[3] = content & 0xFF;

    // command
    data[4] = (id >> 24) & 0xFF;
    data[5] = (id >> 16) & 0xFF;
    data[6] = (id >> 8) & 0xFF;
    data[7] = id & 0xFF;

    CanBusFrame can_frame;
    can_frame.id = CanframeIDs::COMMAND_ID;
    for(size_t i = 0; i < 8; i++)
    {
        can_frame.data[i] = data[i];
    }
    can_frame.dlc = 8;

    can_bus_->set_input_frame(can_frame);
    can_bus_->send_if_input_changed();
}

void CanBusMotorBoard::loop()
{
    enable();

    long int timeindex = can_bus_->get_output_frame()->newest_timeindex();
    while(is_loop_active_)
    {
        CanBusFrame can_frame;
        Index received_timeindex = timeindex;
        can_frame = (*can_bus_->get_output_frame())[received_timeindex];

        if(received_timeindex != timeindex)
        {
            rt_printf("did not get the timeindex we expected! "
                                  "received_timeindex: %d, "
                                  "desired_timeindex: %d\n",
                                  int(received_timeindex), int(timeindex));
            exit(-1);
        }

        timeindex++;

        // convert to measurement ------------------------------------------
        double measurement_0 = qbytes_to_float(can_frame.data.begin());
        double measurement_1 = qbytes_to_float((can_frame.data.begin() + 4));


        switch(can_frame.id)
        {
        case CanframeIDs::Iq:
            measurement_[current_0]->append(measurement_0);
            measurement_[current_1]->append(measurement_1);
            break;
        case CanframeIDs::POS:
            measurement_[position_0]->append(measurement_0);
            measurement_[position_1]->append(measurement_1);
            break;
        case CanframeIDs::SPEED:
            measurement_[velocity_0]->append(measurement_0);
            measurement_[velocity_1]->append(measurement_1);
            break;
        case CanframeIDs::ADC6:
            measurement_[analog_0]->append(measurement_0);
            measurement_[analog_1]->append(measurement_1);
            break;
        case CanframeIDs::ENC_INDEX:
        {
            // here the interpretation of the message is different,
            // we get a motor index and a measurement
            uint8_t motor_index = can_frame.data[4];
            if(motor_index == 0)
            {
                measurement_[encoder_index_0]->append(measurement_0);
            }
            else if(motor_index == 1)
            {
                measurement_[encoder_index_1]->append(measurement_0);
            }
            else
            {
                rt_printf("ERROR: Invalid motor number"
                                      "for encoder index: %d\n", motor_index);
                exit(-1);
            }
            break;
        }
        case CanframeIDs::STATUSMSG:
        {
            MotorBoardStatus status;
            uint8_t data = can_frame.data[0];
            status.system_enabled = data >> 0;
            status.motor1_enabled = data >> 1;
            status.motor1_ready   = data >> 2;
            status.motor2_enabled = data >> 3;
            status.motor2_ready   = data >> 4;
            status.error_code     = data >> 5;

            status_->append(status);
            break;
        }
        }

        // static int count = 0;
        // if(count % 4000 == 0)
        // {
        //     print_status();
        // }
        // count++;
    }
}

void CanBusMotorBoard::print_status()
{
    rt_printf("ouptus ======================================\n");
    rt_printf("measurements: -------------------------------\n");
    for(size_t i = 0; i < measurement_.size(); i++)
    {
        rt_printf("%d: ---------------------------------\n", int(i));
        if(measurement_[i]->length() > 0)
        {
            double measurement = measurement_[i]->newest_element();
            rt_printf("value %f:\n", measurement);
        }
    }

    //        rt_printf("status: ---------------------------------\n");
    //        if(status_[status]->length() > 0)
    //            status_[status]->newest_element().print();

    //        rt_printf("inputs ======================================\n");

    //        for(size_t i = 0; i < control_names.size(); i++)
    //        {
    //            rt_printf("%s: ---------------------------------\n",
    //                                 control_names[i].c_str());
    //            if(control_.at(control_names[i])->length() > 0)
    //            {
    //                double control =
    //                        control_.at(control_names[i])->newest_element();
    //                rt_printf("value %f:\n", control);
    //            }
    //        }

    //        rt_printf("command: ---------------------------------\n");
    //        if(command_[command]->length() > 0)
    //            command_[command]->newest_element().print();
}

} // namespace blmc_drivers
