/**
 * @file motor_board.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <memory>
#include <string>

#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>
#include <time_series/time_series.hpp>

#include "blmc_drivers/devices/can_bus.hpp"
#include "blmc_drivers/devices/device_interface.hpp"
#include "blmc_drivers/utils/os_interface.hpp"

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
    MotorBoardCommand()
    {
    }

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
    uint8_t system_enabled : 1;

    /**
     * @brief Bits 1 enables/disable of the motor 1.
     */
    uint8_t motor1_enabled : 1;  // 1

    /**
     * @brief Bits 2 checks if the motor 1 is ready or not.
     */
    uint8_t motor1_ready : 1;  // 2

    /**
     * @brief Bits 3 enables/disable of the motor 2.
     */
    uint8_t motor2_enabled : 1;  // 3

    /**
     * @brief Bits 4 checks if the motor 2 is ready or not.
     */
    uint8_t motor2_ready : 1;  // 4

    /**
     * @brief This encodes the error codes. See "ErrorCodes" for more details.
     */
    uint8_t error_code : 3;  // 5-7

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
        CRIT_TEMP =
            3,  // currently unused
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

    /**
     * @brief Check if the all status are green.
     */
    bool is_ready() const
    {
        if (system_enabled && motor1_enabled && motor1_ready &&
            motor2_enabled && motor2_ready && !error_code)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //! @brief Get a human-readable description of the error code.
    std::string get_error_description() const
    {
        std::string error_description;
        switch (error_code)
        {
            case ErrorCodes::NONE:
                error_description = "No Error";
                break;
            case ErrorCodes::ENCODER:
                error_description = "Encoder Error";
                break;
            case ErrorCodes::CAN_RECV_TIMEOUT:
                error_description = "CAN Receive Timeout";
                break;
            case ErrorCodes::CRIT_TEMP:
                error_description = "Critical Temperature";
                break;
            case ErrorCodes::POSCONV:
                error_description = "Error in SpinTAC Position Convert module";
                break;
            case ErrorCodes::POS_ROLLOVER:
                error_description = "Position Rollover";
                break;
            case ErrorCodes::OTHER:
                error_description = "Other Error";
                break;
            default:
                error_description = "Unknown Error";
                break;
        }
        return error_description;
    }
};

//==============================================================================
/**
 * @brief MotorBoardInterface declares an API to inacte with a MotorBoard.
 */
class MotorBoardInterface : public DeviceInterface
{
public:
    /**
     * @brief Destroy the MotorBoardInterface object
     */
    virtual ~MotorBoardInterface()
    {
    }

    /**
     * @brief A useful shortcut
     */
    typedef time_series::TimeSeries<double> ScalarTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef time_series::Index Index;
    /**
     * @brief A useful shortcut
     */
    typedef time_series::TimeSeries<Index> IndexTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef time_series::TimeSeries<MotorBoardStatus> StatusTimeseries;
    /**
     * @brief A useful shortcut
     */
    typedef time_series::TimeSeries<MotorBoardCommand> CommandTimeseries;
    /**
     * @brief A useful shortcut
     */
    template <typename Type>
    using Ptr = std::shared_ptr<Type>;
    /**
     * @brief A useful shortcut
     */
    template <typename Type>
    using Vector = std::vector<Type>;

    /**
     * @brief This is the list of the measurement we can access.
     */
    enum MeasurementIndex
    {
        current_0,
        current_1,
        position_0,
        position_1,
        velocity_0,
        velocity_1,
        analog_0,
        analog_1,
        encoder_index_0,
        encoder_index_1,
        measurement_count
    };

    /**
     * @brief This is the list of the controls we can send
     */
    enum ControlIndex
    {
        current_target_0,
        current_target_1,
        control_count
    };

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

/**
 * @brief Create a vector of pointers.
 *
 * @tparam Type of the data
 * @param size is number of pointers to be created.
 * @param length is the dimension of the data arrays.
 * @return Vector<Ptr<Type>> which is the a list of list of data of type
 * Type
 */
template <typename Type>
std::vector<std::shared_ptr<Type>> create_vector_of_pointers(
    const size_t& size, const size_t& length)
{
    std::vector<std::shared_ptr<Type>> vector;
    vector.resize(size);
    for (size_t i = 0; i < size; i++)
    {
        vector[i] = std::make_shared<Type>(length, 0, false);
    }
    return vector;
}

//==============================================================================
/**
 * @brief This class CanBusMotorBoard implements a MotorBoardInterface specific
 * to CAN networks.
 */
class CanBusMotorBoard : public MotorBoardInterface
{
public:
    /**
     * @brief Construct a new CanBusMotorBoard object
     *
     * @param can_bus
     * @param history_length
     */
    CanBusMotorBoard(std::shared_ptr<CanBusInterface> can_bus,
                     const size_t& history_length = 1000,
                     const int& control_timeout_ms = 100);

    /**
     * @brief Destroy the CanBusMotorBoard object
     */
    ~CanBusMotorBoard();

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
    virtual Ptr<const ScalarTimeseries> get_sent_control(const int& index) const
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

    /**
     * @brief returns only once board and motors are ready.
     */
    void wait_until_ready();

    bool is_ready();

    /// \todo: this function should go away,
    /// and we should add somewhere a warning in case there is a timeout
    void pause_motors();

    /**
     * @brief Disable the can reciever timeout.
     */
    void disable_can_recv_timeout();

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
    template <typename T>
    int32_t bytes_to_int32(T bytes)
    {
        return (int32_t)bytes[3] + ((int32_t)bytes[2] << 8) +
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
    template <typename T>
    float qbytes_to_float(T qbytes)
    {
        return q24_to_float(bytes_to_int32(qbytes));
    }

    /**
     * @brief send the controls to the cards.
     *
     * @param controls are the controls to be sent.
     */
    void send_newest_controls();

    /**
     * @brief send the latest commands to the cards.
     *
     */
    void send_newest_command();

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
        return THREAD_FUNCTION_RETURN_VALUE;
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
     * @brief These are the frame IDs that define the kind of data we acquiere
     * from the CAN bus
     */
    enum CanframeIDs
    {
        COMMAND_ID = 0x00,
        IqRef = 0x05,
        STATUSMSG = 0x10,
        Iq = 0x20,
        POS = 0x30,
        SPEED = 0x40,
        ADC6 = 0x50,
        ENC_INDEX = 0x60
    };

    /**
     * Outputs
     */

    /**
     * @brief measurement_ contains all the measurements acquiered from the CAN
     * board.
     */
    Vector<Ptr<ScalarTimeseries>> measurement_;

    /**
     * @brief This is the status history of the CAN board.
     */
    Ptr<StatusTimeseries> status_;

    /**
     * Inputs
     */

    /**
     * @brief This is the buffer of the controls to be sent to card.
     */
    Vector<Ptr<ScalarTimeseries>> control_;

    /**
     * @brief This is the buffer of the commands to be sent to the card.
     */
    Ptr<CommandTimeseries> command_;

    /**
     * Log
     */

    /**
     * @brief This is the history of the already sent controls.
     */
    Vector<Ptr<ScalarTimeseries>> sent_control_;

    /**
     * @brief This is the history of the already sent commands.
     */
    Ptr<CommandTimeseries> sent_command_;

    /**
     * Loop management
     */

    /**
     * @brief This boolean makes sure that the loop is stopped upon destruction
     * of this object.
     */
    bool is_loop_active_;

    /**
     * @brief Are motor in idle mode = 0 torques?
     * @TODO update this documentation with the actual behavior
     */
    bool motors_are_paused_;

    /**
     * @brief If no control is sent for more than control_timeout_ms_ the board
     * will shut down
     */
    int control_timeout_ms_;

    /**
     * @brief This is the thread object that allow to spwan a real-time thread
     * or not dependening on the current OS.
     */
    real_time_tools::RealTimeThread rt_thread_;
};

}  // namespace blmc_drivers
