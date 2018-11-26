/**
 * @file motor_board.hpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This files declares a set of class that encapuslate the network
 * communication with control board. And in particular with can boards.
 * @version 0.1
 * @date 2018-11-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#pragma once

#include <memory>
#include <string>

#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/utils/threadsafe_object.hpp>
#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <real_time_tools/realtime_thread_creation.hpp>

#include <blmc_drivers/utils/os_interface.hpp>
#include <blmc_drivers/devices/can_bus.hpp>

#include <blmc_drivers/devices/device_interface.hpp>


namespace blmc_drivers
{

//==============================================================================
/**
 * @brief This MotorBoardCommand class is a data structurs that defines a
 * command.
 */
class MotorBoardCommand
{
public:

    /**
     * @brief Construct a new MotorBoardCommand object
     */
    MotorBoardCommand() { }

    /**
     * @brief Construct a new MotorBoardCommand object
     * 
     * @param id defines the command to apply.
     * @param content defines of the command is enabled or disabled.
     */
    MotorBoardCommand(uint32_t id, int32_t content)
    {
        id_ = id;
        content_ = content;
    }

    /**
     * @brief Display on a terminal the status of the message.
     */
    void print() const
    {
        rt_printf("command id: %d, content: %d\n", id_, content_);
    }

    /**
     * @brief IDs are the different implemented commands that one can send to 
     * the MotorBoard.
     */
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

    /**
     * @brief Is the different command status.
     */
    enum Contents
    {
        ENABLE = 1,
        DISABLE = 0
    };

    /**
     * @brief id_ is the command to be modifies on the card.
     */
    uint32_t id_;

    /**
     * @brief content_ is the value of teh command to be sent to the cards.
     */
    int32_t content_;
};



//==============================================================================
/**
 * @brief This class represent a 8 bits message that describe the state 
 * (enable/disabled) of the card and the two motors.
 */
class MotorBoardStatus
{
public:
    /**
     * These are the list of bits of the message.
     */
    
    /**
     * @brief Bits 0 enables/disable of the system (motor board).
     */
    uint8_t system_enabled:1;

    /**
     * @brief Bits 1 enables/disable of the motor 1.
     */
    uint8_t motor1_enabled:1;  // 1

    /**
     * @brief Bits 2 checks if the motor 1 is ready or not.
     */
    uint8_t motor1_ready:1;    // 2

    /**
     * @brief Bits 3 enables/disable of the motor 2.
     */
    uint8_t motor2_enabled:1;  // 3

    /**
     * @brief Bits 4 checks if the motor 2 is ready or not.
     */
    uint8_t motor2_ready:1;    // 4

    /**
     * @brief This encodes the error codes. See "ErrorCodes" for more details.
     */
    uint8_t error_code:3;      // 5-7

    /**
     * @brief This is the list of the error codes
     */
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

    /**
     * @brief Simply print the status of the motor board.
     */
    void print() const
    {
        rt_printf("\tSystem enabled: %d\n", system_enabled);
        rt_printf("\tMotor 1 enabled: %d\n", motor1_enabled);
        rt_printf("\tMotor 1 ready: %d\n", motor1_ready);
        rt_printf("\tMotor 2 enabled: %d\n", motor2_enabled);
        rt_printf("\tMotor 2 ready: %d\n", motor2_ready);
        rt_printf("\tError Code: %d\n", error_code);
    }
};



//==============================================================================
/**
 * @brief MotorBoardInterface declares an API to inacte with a MotorBoard.
 */
class MotorBoardInterface: public DeviceInterface
{
public:
    /**
     * @brief Destroy the MotorBoardInterface object
     */
    virtual ~MotorBoardInterface() {}
    
    /**
     * @brief A useful shortcut
     */
    typedef ThreadsafeTimeseries<double> ScalarTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef ScalarTimeseries::Index Index;
    /**
     * @brief A useful shortcut
     */
    typedef ThreadsafeTimeseries<Index> IndexTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef ThreadsafeTimeseries<MotorBoardStatus> StatusTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef ThreadsafeTimeseries<MotorBoardCommand> CommandTimeseries;
    /**
     * @brief A useful shortcut
     */
    template<typename Type> using Ptr = std::shared_ptr<Type>;
    /**
     * @brief A useful shortcut
     */
    template<typename Type> using Vector = std::vector<Type>;

    /**
     * @brief This is the list of the measurement we can access.
     */
    enum MeasurementIndex {current_0, current_1,
                           position_0, position_1,
                           velocity_0, velocity_1,
                           analog_0, analog_1,
                           encoder_index_0, encoder_index_1,
                           measurement_count};

    /**
     * @brief This is the list of the controls we can send
     */
    enum ControlIndex {current_target_0, current_target_1, control_count};

    /**
     * Getters
     */

    /**
     * @brief Get the measurements
     * 
     * @param index is the kind of measurement we are looking for.
     * @return Ptr<const ScalarTimeseries>  is the list of the last time stamped
     * measurement acquiered.
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(
            const int& index) const = 0;

    /**
     * @brief Get the status of the motor board.
     * 
     * @return Ptr<const StatusTimeseries> is the list of the last status of
     * the card.
     */
    virtual Ptr<const StatusTimeseries> get_status() const = 0;

    /**
     * input logs
     */

    /**
     * @brief Get the controls to be send.
     * 
     * @param index define the kind of control we are looking for.
     * @return Ptr<const ScalarTimeseries> is the list of the controls to be
     * send.
     */
    virtual Ptr<const ScalarTimeseries> get_control(const int& index) const = 0;

    /**
     * @brief Get the commands to be send.
     * 
     * @return Ptr<const CommandTimeseries> is the list of the commands to be
     * send.
     */
    virtual Ptr<const CommandTimeseries> get_command() const = 0;

    /**
     * @brief Get the sent controls.
     * 
     * @param index define the kind of control we are looking for.
     * @return Ptr<const ScalarTimeseries> is the list of the controls sent
     * recently.
     */
    virtual Ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const = 0;

    /**
     * @brief Get the sent commands.
     * 
     * @return Ptr<const CommandTimeseries>  is the list of the commands sent
     * recently.
     */
    virtual Ptr<const CommandTimeseries> get_sent_command() const = 0;

    /**
     * Setters
     */
    
    /**
     * @brief set_control save the control internally. In order to actaully send
     * the controls to the network please call "send_if_input_changed"
     * 
     * @param control is the value of the control.
     * @param index define the kind of control we want to send.
     */
    virtual void set_control(const double& control, const int& index) = 0;
    
    /**
     * @brief set_command save the command internally. In order to actaully send
     * the controls to the network please call "send_if_input_changed"
     * 
     * @param command is the command to be sent.
     */
    virtual void set_command(const MotorBoardCommand& command) = 0;

    /**
     * @brief Actually send the commands and the controls
     */
    virtual void send_if_input_changed() = 0;
};



//==============================================================================
/**
 * @brief This class CanBusMotorBoard implements a MotorBoardInterface specific
 * to CAN networks.
 */
class CanBusMotorBoard: public  MotorBoardInterface
{
public:

    /**
     * @brief Construct a new CanBusMotorBoard object
     * 
     * @param can_bus 
     * @param history_length 
     */
    CanBusMotorBoard(std::shared_ptr<CanBusInterface> can_bus,
                     const size_t& history_length = 1000);

    /**
     * @brief Destroy the CanBusMotorBoard object
     */
    ~CanBusMotorBoard();

    /**
     * @brief Create a vector of pointers.
     * 
     * @tparam Type of the data
     * @param size is number of pointers to be created.
     * @param length is the dimension of the data arrays.
     * @return Vector<Ptr<Type>> which is the a list of list of data of type
     * Type
     */
    template<typename Type> Vector<Ptr<Type>> 
    create_vector_of_pointers(const size_t& size, const size_t& length)
    {
        Vector<Ptr<Type>> vector(size);
        for(size_t i = 0; i < size; i++)
        {
            vector[i] = std::make_shared<Type>(length);
        }

        return vector;
    }

    /**
     * Getters
     */

    /**
     * @brief Get the measurement data.
     * 
     * @param index is the kind of measurement we are insterested in.
     * @return Ptr<const ScalarTimeseries> is the list of the last measurements
     * acquiered from the CAN card.
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(const int& index) const
    {
        return measurement_[index];
    }

    /**
     * @brief Get the status of the CAN card.
     * 
     * @return Ptr<const StatusTimeseries> is the list of last acquiered status.
     */
    virtual Ptr<const StatusTimeseries> get_status() const
    {
        return status_;
    }

    /**
     * @brief Get the controls to be sent.
     * 
     * @param index the kind of control we are interested in.
     * @return Ptr<const ScalarTimeseries> is the list of the control to be
     * sent.
     */
    virtual Ptr<const ScalarTimeseries> get_control(const int& index) const
    {
        return control_[index];
    }

    /**
     * @brief Get the commands to be sent.
     * 
     * @return Ptr<const CommandTimeseries> is the list of the command to be
     * sent.
     */
    virtual Ptr<const CommandTimeseries> get_command() const
    {
        return command_;
    }

    /**
     * @brief Get the already sent controls.
     * 
     * @param index the kind of control we are interested in.
     * @return Ptr<const ScalarTimeseries> is the list of the sent cotnrols.
     */
    virtual Ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const
    {
        return control_[index];
    }

    /**
     * @brief Get the already sent commands.
     * 
     * @return Ptr<const CommandTimeseries> is the list of the sent cotnrols.
     */
    virtual Ptr<const CommandTimeseries> get_sent_command() const
    {
        return sent_command_;
    }

    /**
     * Setters
     */

    /**
     * @brief Set the controls, see MotorBoardInterface::set_control
     * 
     * @param control 
     * @param index 
     */
    virtual void set_control(const double& control, const int& index)
    {
        control_[index]->append(control);
    }

    /**
     * @brief Set the commands, see MotorBoardInterface::set_command
     * 
     * @param command 
     */
    virtual void set_command(const MotorBoardCommand& command)
    {
        command_->append(command);
    }

    /**
     * @brief Send the actual command and controls.
     */
    virtual void send_if_input_changed();

    /// private methods ========================================================
private:
    /**
     * Useful converters
     */

    /**
     * @brief Converts from bytes to int32.
     * 
     * @tparam T this is the type of the bytes convert.
     * @param bytes The bytes value
     * @return int32_t the output integer in int32.
     */
    template<typename T> int32_t bytes_to_int32(T bytes)
    {
        return (int32_t) bytes[3] + ((int32_t)bytes[2] << 8) +
                ((int32_t)bytes[1] << 16) + ((int32_t)bytes[0] << 24);
    }

    /**
     * @brief Convert from 24-bit normalized fixed-point to float.
     * 
     * @param qval is the floating base point.
     * @return float is the converted value
     */
    float q24_to_float(int32_t qval)
    {
        return ((float)qval / (1 << 24));
    }

    /**
     * @brief Converts from float to 24-bit normalized fixed-point.
     * 
     * @param fval 
     * @return int32_t 
     */
    int32_t float_to_q24(float fval)
    {
        return ((int)(fval * (1 << 24)));
    }

    /**
     * @brief Converts from qbytes to float
     * 
     * @tparam T the type of byte to manage
     * @param qbytes the input value in bytes
     * @return float the output value.
     */
    template<typename T> float qbytes_to_float(T qbytes)
    {
        return q24_to_float(bytes_to_int32(qbytes));
    }

    /**
     * @brief enable the MotorBoard card.
     * todo: this should go away
     */
    void enable();

    /**
     * @brief append_and_send_command set the command and send it to the cards.
     * 
     * @param command the command to be sent.
     */
    void append_and_send_command(const MotorBoardCommand& command)
    {
        command_->append(command);
        send_if_input_changed();
    }

    /**
     * @brief send the controls to the cards.
     * 
     * @param controls are the controls to be sent.
     */
    void send_controls(std::array<double, 2> controls);

    /**
     * @brief send the commands to the cards.
     * 
     * @param command is the commands to be sent
     */
    void send_command(MotorBoardCommand command);

    /**
     * @brief This is the helper function used for spawning the real time
     * thread.
     * 
     * @param instance_pointer is the current object in this case.
     * @return THREAD_FUNCTION_RETURN_TYPE depends on the current OS.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((CanBusMotorBoard*)(instance_pointer))->loop();
    }

    /**
     * @brief Is the loop that constently communicate with the network.
     */
    void loop();

    /**
     * @brief Display details of this object. 
     */
    void print_status();

private:  

    /**
     * @brief This is the pointer to the can bus to communicate with.
     */
    std::shared_ptr<CanBusInterface> can_bus_;

    /**
     * @brief 
     * 
     */
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

    /// loop management ========================================================
    bool is_loop_active_;
    real_time_tools::RealTimeThread rt_thread_;

};

} // namespace blmc_drivers
