#pragma once

#include <memory>
#include <string>

#include <utils/timer.hpp>
#include <utils/threadsafe_object.hpp>
#include <utils/threadsafe_timeseries.hpp>

#include <utils/os_interface.hpp>
#include <devices/can_bus.hpp>

#include <devices/device_interface.hpp>



class MotorBoardCommand
{
public:
    MotorBoardCommand() { }
    MotorBoardCommand(uint32_t id, int32_t content)
    {
        id_ = id;
        content_ = content;
    }

    void print() const
    {
        osi::printf("command id: %d, content: %d\n", id_, content_);
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



class MotorBoardStatus
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
        osi::printf("\tSystem enabled: %d\n", system_enabled);
        osi::printf("\tMotor 1 enabled: %d\n", motor1_enabled);
        osi::printf("\tMotor 1 ready: %d\n", motor1_ready);
        osi::printf("\tMotor 2 enabled: %d\n", motor2_enabled);
        osi::printf("\tMotor 2 ready: %d\n", motor2_ready);
        osi::printf("\tError Code: %d\n", error_code);
    }
};


class MotorBoardInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    typedef ScalarTimeseries::Index Index;
    typedef ThreadsafeTimeseries<Index> IndexTimeseries;

    typedef ThreadsafeTimeseries<MotorBoardStatus> StatusTimeseries;
    typedef ThreadsafeTimeseries<MotorBoardCommand> CommandTimeseries;

    template<typename Type> using Ptr = std::shared_ptr<Type>;
    template<typename Type> using Vector = std::vector<Type>;


    enum MeasurementIndex {current_0, current_1,
                           position_0, position_1,
                           velocity_0, velocity_1,
                           analog_0, analog_1,
                           encoder_index_0, encoder_index_1,
                           measurement_count};

    enum ControlIndex {current_target_0, current_target_1, control_count};

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_measurement(
            const int& index) const = 0;
    virtual Ptr<const StatusTimeseries> get_status() const = 0;

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_control(const int& index) const = 0;
    virtual Ptr<const CommandTimeseries> get_command() const = 0;
    virtual Ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const = 0;
    virtual Ptr<const CommandTimeseries> get_sent_command() const = 0;

    /// setters ================================================================
    virtual void set_control(const double& control, const int& index) = 0;
    virtual void set_command(const MotorBoardCommand& command) = 0;

    /// sender =================================================================
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~MotorBoardInterface() {}
};






class CanBusMotorBoard: public  MotorBoardInterface
{
public:
    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_measurement(const int& index) const
    {
        return measurement_[index];
    }
    virtual Ptr<const StatusTimeseries> get_status() const
    {
        return status_;
    }

    // input logs --------------------------------------------------------------
    virtual Ptr<const ScalarTimeseries> get_control(const int& index) const
    {
        return control_[index];
    }
    virtual Ptr<const CommandTimeseries> get_command() const
    {
        return command_;
    }
    virtual Ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const
    {
        return control_[index];
    }

    virtual Ptr<const CommandTimeseries> get_sent_command() const
    {
        return sent_command_;
    }

    /// setters ================================================================
    virtual void set_control(const double& control, const int& index)
    {
        control_[index]->append(control);
    }

    virtual void set_command(const MotorBoardCommand& command)
    {
        command_->append(command);
    }

    /// sender =================================================================
    virtual void send_if_input_changed()
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

    /// ========================================================================


private:
    void append_and_send_command(const MotorBoardCommand& command)
    {
        command_->append(command);
        send_if_input_changed();
    }

    /// private members ========================================================
private:
    std::shared_ptr<CanBusInterface> can_bus_;

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
    Vector<Ptr<ScalarTimeseries>> measurement_;
    Ptr<StatusTimeseries> status_;

    /// inputs =================================================================
    Vector<Ptr<ScalarTimeseries>> control_;
    Ptr<CommandTimeseries> command_;

    /// log ====================================================================
    Vector<Ptr<ScalarTimeseries>> sent_control_;
    Ptr<CommandTimeseries> sent_command_;


    /// constructor ============================================================
public:
    template<typename Type> Vector<Ptr<Type>> create_vector_of_pointers(
            const size_t& size,
            const size_t& length)
    {
        Vector<Ptr<Type>> vector(size);
        for(size_t i = 0; i < size; i++)
        {
            vector[i] = std::make_shared<Type>(length);
        }

        return vector;
    }



    CanBusMotorBoard(std::shared_ptr<CanBusInterface> can_bus):
        can_bus_(can_bus)
    {
        measurement_  = create_vector_of_pointers<ScalarTimeseries>(measurement_count, 1000);
        status_       = std::make_shared<StatusTimeseries>(1000);
        control_      = create_vector_of_pointers<ScalarTimeseries>(control_count, 1000);
        command_      = std::make_shared<CommandTimeseries>(1000);
        sent_control_ = create_vector_of_pointers<ScalarTimeseries>(control_count, 1000);
        sent_command_ = std::make_shared<CommandTimeseries>(1000);

        // initialize outputs --------------------------------------------------
        for(size_t i = 0; i < control_.size(); i++)
            control_[i]->append(0);

        osi::start_thread(&CanBusMotorBoard::loop, this);
    }





    ~CanBusMotorBoard()
    {
        append_and_send_command(MotorBoardCommand(MotorBoardCommand::IDs::ENABLE_SYS,
                                                  MotorBoardCommand::Contents::DISABLE));
    }

    /// private methods ========================================================
private:
    // todo: this should go away
    void enable()
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

        CanBusFrame can_frame;
        can_frame.id = BLMC_CAN_ID_IqRef;
        for(size_t i = 0; i < 7; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->set_input_frame(can_frame);
        can_bus_->send_if_input_changed();
    }


    void send_command(MotorBoardCommand command)
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

    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((CanBusMotorBoard*)(instance_pointer))->loop();
    }

    void loop()
    {
        enable();

        long int timeindex = can_bus_->get_output_frame()->newest_timeindex();
        while(true)
        {
            CanBusFrame can_frame;
            Index received_timeindex = timeindex;
            can_frame = (*can_bus_->get_output_frame())[received_timeindex];

            if(received_timeindex != timeindex)
            {
                osi::printf("did not get the timeindex we expected! "
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
                    osi::printf("ERROR: Invalid motor number"
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
        osi::printf("ouptus ======================================\n");
        osi::printf("measurements: -------------------------------\n");
        for(size_t i = 0; i < measurement_.size(); i++)
        {
            osi::printf("%d: ---------------------------------\n", i);
            if(measurement_[i]->length() > 0)
            {
                double measurement = measurement_[i]->newest_element();
                osi::printf("value %f:\n", measurement);
            }
        }

        //        osi::printf("status: ---------------------------------\n");
        //        if(status_[status]->length() > 0)
        //            status_[status]->newest_element().print();

        //        osi::printf("inputs ======================================\n");

        //        for(size_t i = 0; i < control_names.size(); i++)
        //        {
        //            osi::printf("%s: ---------------------------------\n",
        //                                 control_names[i].c_str());
        //            if(control_.at(control_names[i])->length() > 0)
        //            {
        //                double control =
        //                        control_.at(control_names[i])->newest_element();
        //                osi::printf("value %f:\n", control);
        //            }
        //        }

        //        osi::printf("command: ---------------------------------\n");
        //        if(command_[command]->length() > 0)
        //            command_[command]->newest_element().print();
    }

    unsigned id_to_index(unsigned motor_id)
    {
        if(motor_id == BLMC_MTR1)
            return 0;
        else if(motor_id == BLMC_MTR2)
            return 1;

        osi::printf("unknown motor id: %d", motor_id);
        exit(-1);
    }
};
