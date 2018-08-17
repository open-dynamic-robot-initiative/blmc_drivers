#pragma once

#include <memory>
#include <string>

#include <utils/timer.hpp>
#include <utils/threadsafe_object.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <utils/os_interface.hpp>
#include <devices/canbus.hpp>



class MotorboardCommand
{
public:
    MotorboardCommand() { }
    MotorboardCommand(uint32_t id, int32_t content)
    {
        id_ = id;
        content_ = content;
    }

    void print() const
    {
        osi::print_to_screen("command id: %d, content: %d\n", id_, content_);
    }

    enum IDs
    {
        ENABLE_SYS = 1,
        ENABLE_MTR1 = 2,
        ENABLE_MTR2 = 3,
        ENABLE_VSPRING1 = 4,
        ENABLE_VSPRING2 = 5,
        SEND_CURRENT = 12,
        SEND_POSITION = 13,
        SEND_VELOCITY = 14,
        SEND_ADC6 = 15,
        SEND_ENC_INDEX = 16,
        SEND_ALL = 20,
        SET_CAN_RECV_TIMEOUT = 30,
        ENABLE_POS_ROLLOVER_ERROR = 31,
    };

    enum Contents
    {
        ENABLE = 1,
        DISABLE = 0
    };

    uint32_t id_;
    int32_t content_;
};



class MotorboardStatus
{
public:
    // bits
    uint8_t system_enabled:1;  // 0
    uint8_t motor1_enabled:1;  // 1
    uint8_t motor1_ready:1;    // 2
    uint8_t motor2_enabled:1;  // 3
    uint8_t motor2_ready:1;    // 4
    uint8_t error_code:3;      // 5-7

    enum ErrorCodes
    {
        //! \brief No error
        NONE = 0,
        //! \brief Encoder error too high
        ENCODER = 1,
        //! \brief Timeout for receiving current references exceeded
        CAN_RECV_TIMEOUT = 2,
        //! \brief Motor temperature reached critical value
        //! \note This is currently unused as no temperature sensing is done.
        CRIT_TEMP = 3,  // currently unused
        //! \brief Some error in the SpinTAC Position Convert module
        POSCONV = 4,
        //! \brief Position Rollover occured
        POS_ROLLOVER = 5,
        //! \brief Some other error
        OTHER = 7
    };

    void print() const
    {
        osi::print_to_screen("\tSystem enabled: %d\n", system_enabled);
        osi::print_to_screen("\tMotor 1 enabled: %d\n", motor1_enabled);
        osi::print_to_screen("\tMotor 1 ready: %d\n", motor1_ready);
        osi::print_to_screen("\tMotor 2 enabled: %d\n", motor2_enabled);
        osi::print_to_screen("\tMotor 2 ready: %d\n", motor2_ready);
        osi::print_to_screen("\tError Code: %d\n", error_code);
    }
};




template<typename Input, typename Output>
std::map<std::string, Output> copy_map(const std::map<std::string, Input>& input)
{
    std::map<std::string, Output> output;

    for(auto& element : input)
    {
        output[element.first] = element.second;
    }
}





class MotorboardInterface
{
public:
    template<typename Type> using
    MapToPointer = std::map<std::string, std::shared_ptr<Type>>;

    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    typedef ScalarTimeseries::Index Index;
    typedef ThreadsafeTimeseries<Index> IndexTimeseries;

    typedef ThreadsafeTimeseries<MotorboardStatus> StatusTimeseries;
    typedef ThreadsafeTimeseries<MotorboardCommand> CommandTimeseries;

    template<typename Type> using
    Ptr = std::shared_ptr<Type>;

    const std::vector<std::string> measurement_names = {"current_0",
                                                        "current_1",
                                                        "position_0",
                                                        "position_1",
                                                        "velocity_0",
                                                        "velocity_1",
                                                        "analog_0",
                                                        "analog_1",
                                                        "encoder_0",
                                                        "encoder_1"};
    const std::vector<std::string> status_names = {"status"};
    const std::vector<std::string> control_names = {"current_target_0",
                                                    "current_target_1"};
    const std::vector<std::string> command_names = {"command"};

    /// outputs ================================================================
    virtual Ptr<const ScalarTimeseries> measurement(std::string name) const = 0;
    virtual Ptr<const StatusTimeseries> status(std::string name = "status") const = 0;

    /// inputs =================================================================
    virtual Ptr<ScalarTimeseries> control(std::string name) = 0;
    virtual Ptr<CommandTimeseries> command(std::string name = "command") = 0;

    virtual void send_if_input_changed() = 0;

    /// log ====================================================================
    virtual Ptr<const ScalarTimeseries> sent_control(std::string name) = 0;
    virtual Ptr<const CommandTimeseries> sent_command(std::string name = "command") = 0;

    /// ========================================================================

    virtual void print_status() = 0;

    virtual ~MotorboardInterface() {}
};






class CanMotorboard: public  MotorboardInterface
{
public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        return measurement_.at(name);
    }
    virtual std::shared_ptr<const StatusTimeseries> status(std::string name = "status") const
    {
        return status_.at(name);
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> control(std::string name)
    {
        return control_.at(name);
    }
    virtual std::shared_ptr<CommandTimeseries> command(std::string name = "command")
    {
        return command_.at(name);
    }
    virtual void send_if_input_changed()
    {
        // initialize outputs --------------------------------------------------
        bool controls_have_changed = false;

        for(auto element : control_)
        {
            if(element.second->has_changed_since_tag())
                controls_have_changed = true;
        }
        if(controls_have_changed)
        {
            std::array<double, 2> controls_to_send;
            for(size_t i = 0; i < control_names.size(); i++)
            {
                Index timeindex_to_send =
                        control_.at(control_names[i])->newest_timeindex();
                controls_to_send[i] =
                        (*control_.at(control_names[i]))[timeindex_to_send];
                control_.at(control_names[i])->tag(timeindex_to_send);

                sent_control_.at(control_names[i])
                        ->append(controls_to_send[i]);
            }
            send_controls(controls_to_send);
        }

        if(command_.at(command_names[0])->has_changed_since_tag())
        {
            Index timeindex_to_send =
                    command_.at(command_names[0])->newest_timeindex();
            MotorboardCommand command_to_send =
                    (*command_.at(command_names[0]))[timeindex_to_send];
            command_.at(command_names[0])->tag(timeindex_to_send);
            sent_command_.at(command_names[0])
                    ->append(command_to_send);

            send_command(command_to_send);
        }
    }

    /// log ====================================================================
    virtual Ptr<const ScalarTimeseries> sent_control(std::string name)
    {
        return control_.at(name);
    }

    virtual Ptr<const CommandTimeseries> sent_command(std::string name = "command")
    {
        return command_.at(name);
    }

    /// ========================================================================
    // todo: this should go away
    void enable()
    {
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                       MotorboardCommand::Contents::ENABLE));
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::SEND_ALL,
                                       MotorboardCommand::Contents::ENABLE));
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR1,
                                       MotorboardCommand::Contents::ENABLE));
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR2,
                                       MotorboardCommand::Contents::ENABLE));
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::SET_CAN_RECV_TIMEOUT,
                                                      100));
    }

private:
    void append_and_send_command(const MotorboardCommand& command)
    {
        command_.at("command")->append(command);
        send_if_input_changed();
    }

    /// private members ========================================================
private:
    std::shared_ptr<CanbusInterface> can_bus_;

    enum CanframeIDs
    {
        COMMAND_ID= 0x00,
        IqRef     = 0x05,
        STATUSMSG = 0x10,
        Iq        = 0x20,
        POS       = 0x30,
        SPEED     = 0x40,
        ADC6      = 0x50,
        ENC_INDEX = 0x60
    };

    /// outputs ================================================================
    MapToPointer<ScalarTimeseries> measurement_;
    MapToPointer<StatusTimeseries> status_;

    /// inputs =================================================================
    MapToPointer<ScalarTimeseries> control_;
    MapToPointer<CommandTimeseries> command_;

    /// log ====================================================================
    MapToPointer<ScalarTimeseries> sent_control_;
    MapToPointer<CommandTimeseries> sent_command_;


    /// constructor ============================================================
public:
    template<typename Type>
    MapToPointer<Type> create_map(
            const std::vector<std::string>& names,
            const size_t& history_length)
    {
        MapToPointer<Type> map;
        for(size_t i = 0; i < names.size(); i++)
        {
            map[names[i]] =
                    std::make_shared<Type>(history_length);
        }

        return map;
    }



    CanMotorboard(std::shared_ptr<CanbusInterface> can_bus):
        can_bus_(can_bus)
    {
        measurement_  = create_map<ScalarTimeseries>(measurement_names, 1000);
        status_       = create_map<StatusTimeseries>(status_names, 1000);
        control_      = create_map<ScalarTimeseries>(control_names, 1000);
        command_      = create_map<CommandTimeseries>(command_names, 1000);
        sent_control_ = create_map<ScalarTimeseries>(control_names, 1000);
        sent_command_ = create_map<CommandTimeseries>(command_names, 1000);

        // initialize outputs --------------------------------------------------
        for(size_t i = 0; i < control_names.size(); i++)
            control_.at(control_names[i])->append(0);

        osi::start_thread(&CanMotorboard::loop, this);
    }

    ~CanMotorboard()
    {
        append_and_send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                       MotorboardCommand::Contents::DISABLE));
    }

    /// private methods ========================================================
private:
    template<typename T> int32_t bytes_to_int32(T bytes)
    {
        return (int32_t) bytes[3] + ((int32_t)bytes[2] << 8) +
                ((int32_t)bytes[1] << 16) + ((int32_t)bytes[0] << 24);
    }

    float q24_to_float(int32_t qval)
    {
        return ((float)qval / (1 << 24));
    }

    int32_t float_to_q24(float fval)
    {
        return ((int)(fval * (1 << 24)));
    }

    template<typename T> float qbytes_to_float(T qbytes)
    {
        return q24_to_float(bytes_to_int32(qbytes));
    }


    void send_controls(std::array<double, 2> controls)
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

        Canframe can_frame;
        can_frame.id = BLMC_CAN_ID_IqRef;
        for(size_t i = 0; i < 7; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->input_frame()->append(can_frame);
        can_bus_->send_if_input_changed();
    }


    void send_command(MotorboardCommand command)
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

        Canframe can_frame;
        can_frame.id = CanframeIDs::COMMAND_ID;
        for(size_t i = 0; i < 8; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->input_frame()->append(can_frame);
        can_bus_->send_if_input_changed();
    }

    static void
#ifndef __XENO__
    *
#endif
    loop(void* instance_pointer)
    {
        ((CanMotorboard*)(instance_pointer))->loop();
    }

    void loop()
    {

        long int timeindex = can_bus_->output_frame()->next_timeindex();
        while(true)
        {
            Canframe can_frame;
            Index received_timeindex = timeindex;
            can_frame = (*can_bus_->output_frame())[received_timeindex];

            if(received_timeindex != timeindex)
            {
                osi::print_to_screen("did not get the timeindex we expected! "
                                     "received_timeindex: %d, "
                                     "desired_timeindex: %d\n",
                                     received_timeindex, timeindex);
                exit(-1);
            }

            timeindex++;

            // convert to measurement ------------------------------------------
            double measurement_0 = qbytes_to_float(can_frame.data.begin());
            double measurement_1 = qbytes_to_float((can_frame.data.begin() + 4));


            switch(can_frame.id)
            {
            case CanframeIDs::Iq:
                measurement_.at("current_0")->append(measurement_0);
                measurement_.at("current_1")->append(measurement_1);
                break;
            case CanframeIDs::POS:
                measurement_.at("position_0")->append(measurement_0);
                measurement_.at("position_1")->append(measurement_1);
                break;
            case CanframeIDs::SPEED:
                measurement_.at("velocity_0")->append(measurement_0);
                measurement_.at("velocity_1")->append(measurement_1);
                break;
            case CanframeIDs::ADC6:
                measurement_.at("analog_0")->append(measurement_0);
                measurement_.at("analog_1")->append(measurement_1);
                break;
            case CanframeIDs::ENC_INDEX:
            {
                // here the interpretation of the message is different,
                // we get a motor index and a measurement
                uint8_t motor_index = can_frame.data[4];
                if(motor_index == 0)
                {
                    measurement_.at("encoder_0")->append(measurement_0);
                }
                else if(motor_index == 1)
                {
                    measurement_.at("encoder_1")->append(measurement_0);
                }
                else
                {
                    osi::print_to_screen("ERROR: Invalid motor number"
                                         "for encoder index: %d\n", motor_index);
                    exit(-1);
                }
                break;
            }
            case CanframeIDs::STATUSMSG:
            {
                MotorboardStatus status;
                uint8_t data = can_frame.data[0];
                status.system_enabled = data >> 0;
                status.motor1_enabled = data >> 1;
                status.motor1_ready   = data >> 2;
                status.motor2_enabled = data >> 3;
                status.motor2_ready   = data >> 4;
                status.error_code     = data >> 5;

                status_.at("status")->append(status);
                break;
            }
            }

            static int count = 0;
            if(count % 4000 == 0)
            {
                print_status();
            }
            count++;
        }
    }

    void print_status()
    {
        osi::print_to_screen("outputs =====================================\n");

        for(size_t i = 0; i < measurement_names.size(); i++)
        {
            osi::print_to_screen("%s: ---------------------------------\n",
                                 measurement_names[i].c_str());
            if(measurement_.at(measurement_names[i])->history_length() > 0)
            {
                double measurement =
                        measurement_.at(measurement_names[i])->current_element();
                osi::print_to_screen("value %f:\n", measurement);
            }
        }

        osi::print_to_screen("status: ---------------------------------\n");
        if(status_.at("status")->history_length() > 0)
            status_.at("status")->current_element().print();

        osi::print_to_screen("inputs ======================================\n");

        for(size_t i = 0; i < control_names.size(); i++)
        {
            osi::print_to_screen("%s: ---------------------------------\n",
                                 control_names[i].c_str());
            if(control_.at(control_names[i])->history_length() > 0)
            {
                double control =
                        control_.at(control_names[i])->current_element();
                osi::print_to_screen("value %f:\n", control);
            }
        }

        osi::print_to_screen("command: ---------------------------------\n");
        if(command_.at("command")->history_length() > 0)
            command_.at("command")->current_element().print();
    }

    unsigned id_to_index(unsigned motor_id)
    {
        if(motor_id == BLMC_MTR1)
            return 0;
        else if(motor_id == BLMC_MTR2)
            return 1;

        osi::print_to_screen("unknown motor id: %d", motor_id);
        exit(-1);
    }
};
