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

    /// outputs ================================================================
    const MapToPointer<ScalarTimeseries> new_measurement;
    const MapToPointer<StatusTimeseries> new_status;

    /// inputs =================================================================
    const MapToPointer<ScalarTimeseries> new_control;
    const MapToPointer<CommandTimeseries> new_command;

    /// log ====================================================================
    const MapToPointer<ScalarTimeseries> new_sent_control;
    const MapToPointer<IndexTimeseries> new_sent_control_timeindex;
    const MapToPointer<CommandTimeseries> new_sent_command;
    const MapToPointer<IndexTimeseries> new_sent_command_timeindex;


    const MapToPointer<ThreadsafeLoggingTimeseries<double>> sent_control;
    const MapToPointer<ThreadsafeLoggingTimeseries<MotorboardCommand>> sent_command;









    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;
    virtual std::shared_ptr<const StatusTimeseries> status() const = 0;

    /// inputs =================================================================

    virtual std::shared_ptr<ScalarTimeseries> control(std::string name) = 0;
    virtual std::shared_ptr<CommandTimeseries> command() = 0;

    virtual void send_if_input_changed() = 0;

    /// ========================================================================
    virtual void print_status() = 0;

    MotorboardInterface(
            const MapToPointer<ScalarTimeseries>& new_measurement_,
            const MapToPointer<StatusTimeseries>& new_status_,
            const MapToPointer<ScalarTimeseries>& new_control_,
            const MapToPointer<CommandTimeseries>& new_command_,
            const MapToPointer<ScalarTimeseries>& new_sent_control_,
            const MapToPointer<IndexTimeseries>& new_sent_control_timeindex_,
            const MapToPointer<CommandTimeseries>& new_sent_command_,
            const MapToPointer<IndexTimeseries>& new_sent_command_timeindex_,
            const MapToPointer<ThreadsafeLoggingTimeseries<double>>& sent_control_,
            const MapToPointer<ThreadsafeLoggingTimeseries<MotorboardCommand>>& sent_command_):

        new_measurement(new_measurement_),
        new_status(new_status_),
        new_control(new_control_),
        new_command(new_command_),
        new_sent_control(new_sent_control_),
        new_sent_control_timeindex(new_sent_control_timeindex_),
        new_sent_command(new_sent_command_),
        new_sent_command_timeindex(new_sent_command_timeindex_),
        sent_control(sent_control_),
        sent_command(sent_command_) { }

    virtual ~MotorboardInterface() {}


    static const std::vector<std::string> measurement_names;
    static const std::vector<std::string> status_names;
    static const std::vector<std::string> control_names;
    static const std::vector<std::string> command_names;
};
const std::vector<std::string> MotorboardInterface::measurement_names =
{"current_0",
 "current_1",
 "position_0",
 "position_1",
 "velocity_0",
 "velocity_1",
 "analog_0",
 "analog_1",
 "encoder_0",
 "encoder_1"};
const std::vector<std::string> MotorboardInterface::status_names = {"status"};
const std::vector<std::string> MotorboardInterface::control_names =
{"current_target_0",
 "current_target_1"};
const std::vector<std::string> MotorboardInterface::command_names = {"command"};





class CanMotorboard: public  MotorboardInterface
{
    /// public interface =======================================================
public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        return new_measurement.at(name);
    }
    virtual std::shared_ptr<const StatusTimeseries> status() const
    {
        return new_status.at("status");
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> control(std::string name)
    {
        return new_control.at(name);
    }
    virtual std::shared_ptr<CommandTimeseries> command()
    {
        return new_command.at("command");
    }
    virtual void send_if_input_changed()
    {
        // initialize outputs --------------------------------------------------
        bool controls_have_changed = false;
        for(size_t i = 0; i < control_names.size(); i++)
        { 
            if(sent_control.at(control_names[i])
                    ->has_changed(*new_control.at(control_names[i])))
            {
                controls_have_changed = true;
            }

//            if(new_control.at(control_names[i])->history_length() == 0)
//                break;

//            Index current_timeindex =
//                    new_control.at(control_names[i])->next_timeindex() - 1;

//            auto sent_timeindex = new_sent_control_timeindex.at(control_names[i]);
//            if(sent_timeindex->history_length() == 0 ||
//                    sent_timeindex->current_element() < current_timeindex)
//            {
//                sent_timeindex->append(current_timeindex);
//                controls_have_changed = true;
//            }
        }
        if(controls_have_changed)
        {
            send_controls();
        }

        bool commands_have_changed = false;
        for(size_t i = 0; i < command_names.size(); i++)
        {
            if(new_command.at(command_names[i])->history_length() == 0)
                break;

            Index current_timeindex =
                    new_command.at(command_names[i])->next_timeindex() - 1;

            auto sent_timeindex = new_sent_command_timeindex.at(command_names[i]);
            if(sent_timeindex->history_length() == 0 ||
                    sent_timeindex->current_element() < current_timeindex)
            {
                sent_timeindex->append(current_timeindex);
                commands_have_changed = true;
            }
        }
        if(commands_have_changed)
        {
            send_command();
        }
    }

    /// ========================================================================
    // todo: this should go away
    void enable()
    {
        send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                       MotorboardCommand::Contents::ENABLE));
        send_command(MotorboardCommand(MotorboardCommand::IDs::SEND_ALL,
                                       MotorboardCommand::Contents::ENABLE));
        send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR1,
                                       MotorboardCommand::Contents::ENABLE));
        send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR2,
                                       MotorboardCommand::Contents::ENABLE));
        send_command(MotorboardCommand(MotorboardCommand::IDs::SET_CAN_RECV_TIMEOUT,
                                                      100));
    }

private:
    void send_command(const MotorboardCommand& command)
    {
        new_command.at("command")->append(command);
        send_if_input_changed();
    }

    /// private members ========================================================
private:
    std::shared_ptr<CanbusInterface> can_bus_;

    SingletypeThreadsafeObject<long int, 1> command_hash_;

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
        can_bus_(can_bus),
        MotorboardInterface(create_map<ScalarTimeseries>(measurement_names, 1000),
                            create_map<StatusTimeseries>(status_names, 1000),
                            create_map<ScalarTimeseries>(control_names, 1000),
                            create_map<CommandTimeseries>(command_names, 1000),
                            create_map<ScalarTimeseries>(control_names, 1000),
                            create_map<IndexTimeseries>(control_names, 1000),
                            create_map<CommandTimeseries>(command_names, 1000),
                            create_map<IndexTimeseries>(command_names, 1000),
                            create_map<ThreadsafeLoggingTimeseries<double>>(control_names, 1000),
                            create_map<ThreadsafeLoggingTimeseries<MotorboardCommand>>(command_names, 1000))

    {
        // initialize outputs --------------------------------------------------
        for(size_t i = 0; i < control_names.size(); i++)
        {
            new_control.at(control_names[i])->append(0);
        }
        command_hash_.set(new_command.at("command")->next_timeindex());

        osi::start_thread(&CanMotorboard::loop, this);
    }

    ~CanMotorboard()
    {
        send_command(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
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

    void send_controls()
    {
        for(auto element : sent_control)
        {
            element.second->update_if_changed(*new_control.at(element.first));
        }


        float current_mtr1 = new_control.at("current_target_0")->current_element();
        float current_mtr2 = new_control.at("current_target_1")->current_element();

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
    void send_command()
    {
        MotorboardCommand command = new_command.at("command")->current_element();

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

//            osi::print_to_screen("waiting for can frame with index %d\n", timeindex);
            Canframe can_frame;
            Index received_timeindex;
            std::tie(can_frame, received_timeindex)
                    = (*can_bus_->output_frame())[timeindex];


            if(received_timeindex != timeindex)
            {
                osi::print_to_screen("did not get the timeindex we expected! "
                                     "received_timeindex: %d, "
                                     "desired_timeindex: %d\n",
                                     received_timeindex, timeindex);

                exit(-1);
            }

            timeindex++;
//            osi::print_to_screen("received\n");

            // convert to measurement ------------------------------------------
            double measurement_0 = qbytes_to_float(can_frame.data.begin());
            double measurement_1 = qbytes_to_float((can_frame.data.begin() + 4));


            switch(can_frame.id)
            {
            case CanframeIDs::Iq:
                new_measurement.at("current_0")->append(measurement_0);
                new_measurement.at("current_1")->append(measurement_1);
                break;
            case CanframeIDs::POS:
                new_measurement.at("position_0")->append(measurement_0);
                new_measurement.at("position_1")->append(measurement_1);
                break;
            case CanframeIDs::SPEED:
                new_measurement.at("velocity_0")->append(measurement_0);
                new_measurement.at("velocity_1")->append(measurement_1);
                break;
            case CanframeIDs::ADC6:
                new_measurement.at("analog_0")->append(measurement_0);
                new_measurement.at("analog_1")->append(measurement_1);
                break;
            case CanframeIDs::ENC_INDEX:
            {
                // here the interpretation of the message is different,
                // we get a motor index and a measurement
                uint8_t motor_index = can_frame.data[4];
                if(motor_index == 0)
                {
                    new_measurement.at("encoder_0")->append(measurement_0);
                }
                else if(motor_index == 1)
                {
                    new_measurement.at("encoder_1")->append(measurement_0);
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

                new_status.at("status")->append(status);
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
            if(new_measurement.at(measurement_names[i])->history_length() > 0)
            {
                double measurement =
                        new_measurement.at(measurement_names[i])->current_element();
                osi::print_to_screen("value %f:\n", measurement);
            }
        }

        osi::print_to_screen("status: ---------------------------------\n");
        if(new_status.at("status")->history_length() > 0)
            new_status.at("status")->current_element().print();

        osi::print_to_screen("inputs ======================================\n");

        for(size_t i = 0; i < control_names.size(); i++)
        {
            osi::print_to_screen("%s: ---------------------------------\n",
                                 control_names[i].c_str());
            if(new_control.at(control_names[i])->history_length() > 0)
            {
                double control =
                        new_control.at(control_names[i])->current_element();
                osi::print_to_screen("value %f:\n", control);
            }
        }

        osi::print_to_screen("command: ---------------------------------\n");
        if(new_command.at("command")->history_length() > 0)
            new_command.at("command")->current_element().print();
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
