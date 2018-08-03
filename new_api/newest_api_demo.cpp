/**
 * \brief Demo for the blmc_can library.
 *
 * This file shows how the blmc_can library can be used.  At start-up it sends
 * a few commands to the board to enable it and to make it send motor
 * information.  Then messages from the board (and optionally the OptoForce
 * sensor) are received and printed in a loop.
 *
 * While it has been changed in wide parts, this demo is originally based on
 * the rtcanrecv.c example file of xenomai (see copyright and licence below).
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/cond.h>

#include <rtdk.h>
#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>
#include <blmc_can/optoforce_can.h>
#include <functional>


#include <iostream>
#include <stdlib.h>
#include <memory>

#include <eigen3/Eigen/Core>



#include <array>
#include <time_logger.hpp>


#include <tuple>

#include <threadsafe_object.hpp>

#include <string>
#include <iostream>

#include <os_interface.hpp>


//long unsigned count_xenomai_mode_switches = 0;


//// TODO: make sure that mode switcheds are actually detected
//void action_upon_switch(int sig __attribute__((unused)))
//{
//  void *bt[32];
//  int nentries;

//  // increment mode swich counter
//  ++count_xenomai_mode_switches;

//  rt_printf("MOOOOODE SWIIIIITCH\n");
//  exit(-1);
//}

//// GLOBALS
//// **************************************************************************
//std::vector<CAN_CanConnection_t> can_connections;


// FUNCTIONS
// **************************************************************************
//void cleanup_and_exit(int sig)
//{
//    rt_printf("Signal %d received\n", sig);
//    // Disable system before closing connection

//    for(int i = 0; i < can_connections.size(); i++)
//    {
//        BLMC_sendCommand(&can_connections[i], BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
//        CAN_closeCan(&can_connections[i]);
//    }
//    exit(0);
//}







// new class created to replace CAN_Frame_t,
// avoiding dangerous pointers
class CanFrame
{
public:
    std::array<uint8_t, 8> data;
    uint8_t dlc;
    can_id_t id;
};

// new class created to replace _CAN_CanConnection_t_,
// avoiding dangerous pointers, and unnecessary information
class CanConnection
{
public:
    struct sockaddr_can send_addr;
    int socket;
};





template<typename DataType> class StampedData
{
    /// public interface ===========================================================
public:
    // getters -----------------------------------------------------------------
    const DataType& get_data() const
    {
        return data_;
    }
    const size_t& get_id() const
    {
        return id_;
    }
    const double& get_time_stamp() const
    {
        return time_stamp_;
    }

    // setter ------------------------------------------------------------------
    void set_data(const DataType& data)
    {
        data_ = data;
    }

    template<typename Type>
    void set_header_equal(const StampedData<Type>& other_stamped_data)
    {
        id_ = other_stamped_data.get_id();
        time_stamp_ = other_stamped_data.get_time_stamp();
    }

    // constructors ------------------------------------------------------------
    StampedData():
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    StampedData(const DataType& data):
        data_(data),
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    StampedData(const DataType& data,
                const size_t& id,
                const double& time_stamp):
        data_(data),
        id_(id),
        time_stamp_(time_stamp) {  }

    template<typename Type>
    StampedData(const StampedData<Type>& other_stamped_data,
                const DataType& data)
    {
        data_ = data;
        set_header_equal(other_stamped_data);
    }

    /// private data ===========================================================
private:
    DataType data_;
    size_t id_;
    double time_stamp_;
};









// Convertion of a byte array to a int32_t.
#define BYTES_TO_INT32(bytes) (\
    (int32_t) bytes[3] + \
    ((int32_t)bytes[2] << 8) + \
    ((int32_t)bytes[1] << 16) + \
    ((int32_t)bytes[0] << 24) \
    )

// Conversion of Q24 value to float.
#define Q24_TO_FLOAT(qval) ((float)qval / (1 << 24))

// Conversion of float to Q24
#define FLOAT_TO_Q24(fval) ((int)(fval * (1 << 24)))

// Convertion of Q24 byte array to float.
#define QBYTES_TO_FLOAT(qbytes) (\
    Q24_TO_FLOAT( BYTES_TO_INT32(qbytes) ) )



/// a device has to
///  - keep output object up to data
///  - keep input object up to data
///  - implement send functions
///  we can split it up into an input and an output device, and then derive from both for Input Output objects
///  send(bla, input_id)




// This interface allows to communicate with the can bus. It has one input
// (can_frame) and one output (can_frame).
class CanbusInterface
{
public:
    typedef StampedData<CanFrame> StampedFrame;

    // get output data ---------------------------------------------------------
    virtual StampedFrame output_get_can_frame() const = 0;
    virtual void output_wait_for_can_frame() const = 0;
    virtual size_t output_wait_for_any() const = 0;

    // get input data ----------------------------------------------------------
    virtual StampedFrame input_get_can_frame() const = 0;
    virtual void input_wait_for_can_frame() const = 0;
    virtual size_t input_wait_for_any() const = 0;

    // send input data ---------------------------------------------------------
    virtual void input_send_can_frame(const StampedFrame& stamped_can_frame) = 0;

    virtual ~CanbusInterface() {}
};



class XenomaiCanbus: public CanbusInterface
{
    /// public interface ===========================================================
public:
    typedef StampedData<CanFrame> StampedFrame;

    // get output data ---------------------------------------------------------
    StampedFrame output_get_can_frame() const
    {
        return output_.get();
    }
    void output_wait_for_can_frame() const
    {
        output_.wait_for_update();
    }

    size_t output_wait_for_any() const
    {
        return output_.wait_for_update();
    }

    // get input data ----------------------------------------------------------
    StampedFrame input_get_can_frame() const
    {
        return input_.get();
    }
    void input_wait_for_can_frame() const
    {
        input_.wait_for_update();
    }

    size_t input_wait_for_any() const
    {
        return input_.wait_for_update();
    }

    // send input data ---------------------------------------------------------
    void input_send_can_frame(const StampedFrame& stamped_can_frame)
    {
        input_.set(stamped_can_frame);

        // get address ---------------------------------------------------------
        int socket = connection_info_.get().socket;
        struct sockaddr_can address = connection_info_.get().send_addr;


        auto unstamped_can_frame = stamped_can_frame.get_data();
        // put data into can frame ---------------------------------------------
        can_frame_t can_frame;
        can_frame.can_id = unstamped_can_frame.id;
        can_frame.can_dlc = unstamped_can_frame.dlc;


        memcpy(can_frame.data, unstamped_can_frame.data.begin(), unstamped_can_frame.dlc);

        // send ----------------------------------------------------------------
        osi::send_to_can_device(socket,
                                (void *)&can_frame,
                                sizeof(can_frame_t),
                                0,
                                (struct sockaddr *)&address,
                                sizeof(address));

    }

    // constructor and destructor ----------------------------------------------
    XenomaiCanbus(std::string can_interface_name)
    {

        // setup can connection --------------------------------
        // \todo get rid of old format stuff
        CAN_CanConnection_t can_connection_old_format;
        int ret = CAN_setupCan(&can_connection_old_format,
                               can_interface_name.c_str(), 0);
        if (ret < 0)
        {
            osi::print_to_screen("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }

        // \todo:how do we make sure that can connection is closed when we close
        //        can_connections.push_back(can_connection_old_format);

        CanConnection can_connection;

        can_connection.send_addr = can_connection_old_format.send_addr;
        can_connection.socket = can_connection_old_format.socket;

        connection_info_.set(can_connection);


        osi::start_thread(&XenomaiCanbus::loop, this);
    }
    virtual ~XenomaiCanbus()
    {
        osi::close_can_device(connection_info_.get().socket);
    }

    /// private attributes and methods =========================================
private:
    // attributes --------------------------------------------------------------
    ThreadsafeObject<CanConnection> connection_info_;
    ThreadsafeObject<StampedFrame> output_;
    ThreadsafeObject<StampedFrame> input_;

    // methods -----------------------------------------------------------------
    static void loop(void* instance_pointer)
    {
        ((XenomaiCanbus*)(instance_pointer))->loop();
    }

    void loop()
    {
        Timer<100> loop_time_logger("can bus loop", 4000);
        Timer<100> receive_time_logger("receive", 4000);

        while (true)
        {
            receive_time_logger.start_interval();
            CanFrame frame = receive_frame();
            receive_time_logger.end_interval();

            output_.set<0>(StampedData<CanFrame>(
                               frame,
                               output_.get<0>().get_id() + 1,
                               Timer<1>::current_time_ms()));

            loop_time_logger.end_and_start_interval();
        }
    }

    CanFrame receive_frame()
    {
        int socket = connection_info_.get().socket;

        // data we want to obtain ----------------------------------------------
        can_frame_t can_frame;
        nanosecs_abs_t timestamp;
        struct sockaddr_can message_address;

        // setup message such that data can be received to variables above -----
        struct iovec input_output_vector;
        input_output_vector.iov_base = (void *)&can_frame;
        input_output_vector.iov_len = sizeof(can_frame_t);

        struct msghdr message_header;
        message_header.msg_iov = &input_output_vector;
        message_header.msg_iovlen = 1;
        message_header.msg_name = (void *)&message_address;
        message_header.msg_namelen = sizeof(struct sockaddr_can);
        message_header.msg_control = (void *)&timestamp;
        message_header.msg_controllen = sizeof(nanosecs_abs_t);

        // receive message from can bus ----------------------------------------
        osi::receive_message_from_can_device(socket, &message_header, 0);

        // process received data and put into felix widmaier's format ----------
        if (message_header.msg_controllen == 0)
        {
            // No timestamp for this frame available. Make sure we dont get
            // garbage.
            timestamp = 0;
        }

        CanFrame out_frame;
        out_frame.id = can_frame.can_id;
        out_frame.dlc = can_frame.can_dlc;
        for(size_t i = 0; i < can_frame.can_dlc; i++)
        {
            out_frame.data[i] = can_frame.data[i];
        }

        return out_frame;
    }
};






//class XenomaiCanMotorboardInput: public ThreadsafeObject
//{

//};

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


class MotorboardCommand
{
public:
    MotorboardCommand()
    {

    }


    MotorboardCommand(uint32_t id, int32_t content)
    {
        id_ = id;
        content_ = content;
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




class MotorboardInterface
{
public:
    typedef StampedData<double> StampedScalar;
    typedef StampedData<MotorboardCommand> StampedCommand;
    typedef StampedData<_BLMC_StatusMsg_t_> StampedStatus;

    enum InputNames {
        CURRENT_TARGET_A,
        CURRENT_TARGET_B,
        COMMAND
    };

    enum OutputNames {
        CURRENT_A,
        CURRENT_B,
        POSITION_A,
        POSITION_B,
        VELOCITY_A,
        VELOCITY_B,
        ANALOG_A,
        ANALOG_B,
        ENCODER_A,
        ENCODER_B,
        STATUS};

public:
    // get output data ---------------------------------------------------------
    virtual StampedScalar output_get_current_a() const = 0;
    virtual StampedScalar output_get_current_b() const = 0;
    virtual StampedScalar output_get_position_a() const = 0;
    virtual StampedScalar output_get_position_b() const = 0;
    virtual StampedScalar output_get_velocity_a() const = 0;
    virtual StampedScalar output_get_velocity_b() const = 0;
    virtual StampedScalar output_get_analog_a() const = 0;
    virtual StampedScalar output_get_analog_b() const = 0;
    virtual StampedScalar output_get_encoder_a() const = 0;
    virtual StampedScalar output_get_encoder_b() const = 0;
    virtual StampedStatus output_get_status() const = 0;

    virtual void output_wait_for_current_a() const = 0;
    virtual void output_wait_for_current_b() const = 0;
    virtual void output_wait_for_position_a() const = 0;
    virtual void output_wait_for_position_b() const = 0;
    virtual void output_wait_for_velocity_a() const = 0;
    virtual void output_wait_for_velocity_b() const = 0;
    virtual void output_wait_for_analog_a() const = 0;
    virtual void output_wait_for_analog_b() const = 0;
    virtual void output_wait_for_encoder_a() const = 0;
    virtual void output_wait_for_encoder_b() const = 0;
    virtual void output_wait_for_status() const = 0;

    virtual size_t output_wait_for_any() const = 0;

    // get input data ----------------------------------------------------------
    virtual StampedScalar input_get_current_target_a() const = 0;
    virtual StampedScalar input_get_current_target_b() const = 0;
    virtual StampedCommand input_get_command() const = 0;

    virtual void input_wait_for_current_target_a() const = 0;
    virtual void input_wait_for_current_target_b() const = 0;
    virtual void input_wait_for_command() const = 0;

    virtual size_t input_wait_for_any() const = 0;

    // send input data ---------------------------------------------------------
    virtual void input_send_current_target_a(StampedScalar current_target) = 0;
    virtual void input_send_current_target_b(StampedScalar current_target) = 0;
    virtual void input_send_command(const StampedCommand& command) = 0;

    virtual ~MotorboardInterface() {}
};










class XenomaiCanMotorboard: public MotorboardInterface
{
    /// public interface =======================================================
public:
    // get output data ---------------------------------------------------------
    StampedScalar output_get_current_a() const
    {
        return output_.get<CURRENT_A>();
    }
    StampedScalar output_get_current_b() const
    {
        return output_.get<CURRENT_B>();
    }
    StampedScalar output_get_position_a() const
    {
        return output_.get<POSITION_A>();
    }
    StampedScalar output_get_position_b() const
    {
        return output_.get<POSITION_B>();
    }
    StampedScalar output_get_velocity_a() const
    {
        return output_.get<VELOCITY_A>();
    }
    StampedScalar output_get_velocity_b() const
    {
        return output_.get<VELOCITY_B>();
    }
    StampedScalar output_get_analog_a() const
    {
        return output_.get<ANALOG_A>();
    }
    StampedScalar output_get_analog_b() const
    {
        return output_.get<ANALOG_B>();
    }
    StampedScalar output_get_encoder_a() const
    {
        return output_.get<ENCODER_A>();
    }
    StampedScalar output_get_encoder_b() const
    {
        return output_.get<ENCODER_B>();
    }
    StampedStatus output_get_status() const
    {
        return output_.get<STATUS>();
    }

    void output_wait_for_current_a() const
    {
        output_.wait_for_update(CURRENT_A);
    }
    void output_wait_for_current_b() const
    {
        output_.wait_for_update(CURRENT_B);
    }
    void output_wait_for_position_a() const
    {
        output_.wait_for_update(POSITION_A);
    }
    void output_wait_for_position_b() const
    {
        output_.wait_for_update(POSITION_B);
    }
    void output_wait_for_velocity_a() const
    {
        output_.wait_for_update(VELOCITY_A);
    }
    void output_wait_for_velocity_b() const
    {
        output_.wait_for_update(VELOCITY_B);
    }
    void output_wait_for_analog_a() const
    {
        output_.wait_for_update(ANALOG_A);
    }
    void output_wait_for_analog_b() const
    {
        output_.wait_for_update(ANALOG_B);
    }
    void output_wait_for_encoder_a() const
    {
        output_.wait_for_update(ENCODER_A);
    }
    void output_wait_for_encoder_b() const
    {
        output_.wait_for_update(ENCODER_B);
    }
    void output_wait_for_status() const
    {
        output_.wait_for_update(STATUS);
    }

    size_t output_wait_for_any() const
    {
        return output_.wait_for_update();
    }

    // get input data ----------------------------------------------------------
    StampedScalar input_get_current_target_a() const
    {
        return input_.get<CURRENT_TARGET_A>();
    }
    StampedScalar input_get_current_target_b() const
    {
        return input_.get<CURRENT_TARGET_B>();
    }
    StampedCommand input_get_command() const
    {
        return input_.get<COMMAND>();
    }

    void input_wait_for_current_target_a() const
    {
        input_.wait_for_update(CURRENT_TARGET_A);
    }
    void input_wait_for_current_target_b() const
    {
        input_.wait_for_update(CURRENT_TARGET_B);
    }
    void input_wait_for_command() const
    {
        input_.wait_for_update(COMMAND);
    }

    size_t input_wait_for_any() const
    {
        return input_.wait_for_update();
    }

    // send input data ---------------------------------------------------------
    void input_send_current_target_a(StampedScalar current_target)
    {
        input_.set<CURRENT_TARGET_A>(current_target);
        send_current_targets();
    }

    void input_send_current_target_b(StampedScalar current_target)
    {
        input_.set<CURRENT_TARGET_B>(current_target);
        send_current_targets();
    }


    void input_send_command(const StampedCommand& command)
    {
        input_.set<COMMAND>(command);

        uint32_t id = command.get_data().id_;
        int32_t content = command.get_data().content_;


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

        CanFrame can_frame;
        can_frame.id = BLMC_CAN_ID_COMMAND;
        for(size_t i = 0; i < 8; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->input_send_can_frame(StampedData<CanFrame>(can_frame,
                                                             -1, -1));
    }

    // todo: this should go away
    void enable()
    {
        input_send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                                            MotorboardCommand::Contents::ENABLE),-1,-1));
        input_send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::SEND_ALL,
                                                            MotorboardCommand::Contents::ENABLE),-1,-1));
        input_send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR1,
                                                            MotorboardCommand::Contents::ENABLE),-1,-1));
        input_send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR2,
                                                            MotorboardCommand::Contents::ENABLE),-1,-1));
    }

    /// private members ========================================================
private:
    std::shared_ptr<XenomaiCanbus> can_bus_;

    ThreadsafeObject<
    StampedScalar,
    StampedScalar,
    StampedCommand> input_;

    ThreadsafeObject<
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedScalar,
    StampedStatus > output_;

    /// constructor ============================================================
public:
    XenomaiCanMotorboard(std::shared_ptr<XenomaiCanbus> can_bus): can_bus_(can_bus)
    {


        // initialize members ------------------------------------------------------
        StampedScalar stamped_default_measurement(0, -1, -1);
        output_.set<CURRENT_A>(stamped_default_measurement);
        output_.set<CURRENT_B>(stamped_default_measurement);
        output_.set<POSITION_A>(stamped_default_measurement);
        output_.set<POSITION_B>(stamped_default_measurement);
        output_.set<VELOCITY_A>(stamped_default_measurement);
        output_.set<VELOCITY_B>(stamped_default_measurement);
        output_.set<ANALOG_A>(StampedScalar(0.5, -1, -1));
        output_.set<ANALOG_B>(StampedScalar(0.5, -1, -1));
        output_.set<ENCODER_A>(stamped_default_measurement);
        output_.set<ENCODER_B>(stamped_default_measurement);

        input_.set<CURRENT_TARGET_A>(stamped_default_measurement);
        input_.set<CURRENT_TARGET_B>(stamped_default_measurement);

        _BLMC_StatusMsg_t_ default_status_message;
        default_status_message.system_enabled = 0;
        default_status_message.motor1_enabled = 0;
        default_status_message.motor1_ready   = 0;
        default_status_message.motor2_enabled = 0;
        default_status_message.motor2_ready   = 0;
        default_status_message.error_code     = 0;
        output_.set<STATUS>(
                    StampedData<_BLMC_StatusMsg_t_>(default_status_message,
                                                    -1, -1));

        osi::start_thread(&XenomaiCanMotorboard::loop, this);
    }

    /// private methods ========================================================
private:
    void send_current_targets()
    {
        Eigen::Vector2d current_targets;
        current_targets[0] = input_.get<CURRENT_TARGET_A>().get_data();
        current_targets[1] = input_.get<CURRENT_TARGET_B>().get_data();

        float current_mtr1 = current_targets[0];
        float current_mtr2 = current_targets[1];

        uint8_t data[8];
        uint32_t q_current1, q_current2;

        // Convert floats to Q24 values
        q_current1 = FLOAT_TO_Q24(current_mtr1);
        q_current2 = FLOAT_TO_Q24(current_mtr2);

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

        CanFrame can_frame;
        can_frame.id = BLMC_CAN_ID_IqRef;
        for(size_t i = 0; i < 7; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        return can_bus_->input_send_can_frame(StampedData<CanFrame>(can_frame, -1, -1));
    }

    static void loop(void* instance_pointer)
    {
        ((XenomaiCanMotorboard*)(instance_pointer))->loop();
    }

    void loop()
    {

        while(true)
        {


            // get latest can frame --------------------------------------------

            can_bus_->output_wait_for_any();




            auto stamped_can_frame = can_bus_->output_get_can_frame();
            auto can_frame = stamped_can_frame.get_data();

            // convert to measurement ------------------------------------------
            Eigen::Vector2d measurement;
            double measurement_a = QBYTES_TO_FLOAT(can_frame.data.begin());
            double measurement_b =
                    QBYTES_TO_FLOAT((can_frame.data.begin() + 4));

            StampedScalar
                    stamped_measurement_a(measurement_a,
                                          stamped_can_frame.get_id(),
                                          stamped_can_frame.get_time_stamp());
            StampedScalar
                    stamped_measurement_b(measurement_b,
                                          stamped_can_frame.get_id(),
                                          stamped_can_frame.get_time_stamp());



            switch(can_frame.id)
            {
            case BLMC_CAN_ID_Iq:
                output_.set<CURRENT_A>(stamped_measurement_a);
                output_.set<CURRENT_B>(stamped_measurement_b);
                break;
            case BLMC_CAN_ID_POS:
                output_.set<POSITION_A>(stamped_measurement_a);
                output_.set<POSITION_B>(stamped_measurement_b);
                break;
            case BLMC_CAN_ID_SPEED:
                output_.set<VELOCITY_A>(stamped_measurement_a);
                output_.set<VELOCITY_B>(stamped_measurement_b);
                break;
            case BLMC_CAN_ID_ADC6:
                output_.set<ANALOG_A>(stamped_measurement_a);
                output_.set<ANALOG_B>(stamped_measurement_b);
                break;
            case BLMC_CAN_ID_ENC_INDEX:
            {
                // here the interpretation of the message is different,
                // we get a motor index and a measurement
                uint8_t motor_index = can_frame.data[4];
                StampedScalar measurement = stamped_measurement_a;
                if(motor_index == 0)
                {
                    output_.set<ENCODER_A>(measurement);
                }
                else if(motor_index == 1)
                {
                    output_.set<ENCODER_B>(measurement);
                }

                else
                {
                    osi::print_to_screen("ERROR: Invalid motor number"
                              "for encoder index: %d\n", motor_index);
                    exit(-1);
                }
                break;
            }
            case BLMC_CAN_ID_STATUSMSG:
            {
                _BLMC_StatusMsg_t_ status;
                uint8_t data = can_frame.data[0];
                status.system_enabled = data >> 0;
                status.motor1_enabled = data >> 1;
                status.motor1_ready   = data >> 2;
                status.motor2_enabled = data >> 3;
                status.motor2_ready   = data >> 4;
                status.error_code     = data >> 5;

                StampedData<_BLMC_StatusMsg_t_>
                        stamped_status(status,
                                       stamped_can_frame.get_id(),
                                       stamped_can_frame.get_time_stamp());

                output_.set<STATUS>(stamped_status);
                break;
            }
            }

            static int count = 0;
            if(count % 4000 == 0)
            {
                print_everything();



                //                BLMC_printSensorData(&bd->latest);
                //                BLMC_printEncoderIndex(bd->encoder_index);


                //                BLMC_printLatestBoardStatus(&data_);
            }
            count++;

        }
    }


    void print_everything()
    {
        auto status = output_.get<STATUS>();
        osi::print_to_screen("status: time_stamp = %f, id = %d ---------------\n", status.get_time_stamp(), status.get_id());
        BLMC_printStatus(&status.get_data());

        //        auto encoders = output_.get<ENCODERS>();
        //        osi::print_to_screen("encoders: time_stamp = %f, id = %d ---------------\n", encoders.get_time_stamp(), encoders.get_id());
        //        std::stringstream output_string;
        //        output_string << encoders.get_data().transpose();
        //        osi::print_to_screen("%s\n", output_string.str().c_str());
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


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class Motor
{
    // \todo: should probably make this a shared pointer
    std::shared_ptr<XenomaiCanMotorboard> board_;
    bool motor_id_;
public:

    Motor(std::shared_ptr<XenomaiCanMotorboard> board, unsigned motor_id):
        board_(board), motor_id_(motor_id) { }

    double get_latest_currents()
    {
        if(motor_id_ == 0)
            return board_->output_get_current_a().get_data();
        else
            return board_->output_get_current_b().get_data();
    }
    double get_latest_positions()
    {
        if(motor_id_ == 0)
            return board_->output_get_position_a().get_data();
        else
            return board_->output_get_position_b().get_data();
    }
    double get_latest_velocities()
    {
        if(motor_id_ == 0)
            return board_->output_get_velocity_a().get_data();
        else
            return board_->output_get_velocity_b().get_data();
    }
    double get_latest_encoders()
    {
        if(motor_id_ == 0)
            return board_->output_get_encoder_a().get_data();
        else
            return board_->output_get_encoder_b().get_data();
    }

    void set_current_target(double current_target)
    {
        if(motor_id_ == 0)
            return board_->input_send_current_target_a(StampedData<double>(current_target, -1, -1));
        else
            return board_->input_send_current_target_b(StampedData<double>(current_target, -1, -1));
    }
};




class AnalogSensor
{
    std::shared_ptr<XenomaiCanMotorboard> board_;
    bool sensor_id_;
public:

    AnalogSensor(std::shared_ptr<XenomaiCanMotorboard> board, unsigned sensor_id):
        board_(board), sensor_id_(sensor_id) { }

    double get_latest_analogs()
    {
        if(sensor_id_ == 0)
            return board_->output_get_analog_a().get_data();
        else
            return board_->output_get_analog_b().get_data();
    }
};



class Controller
{
private:
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<AnalogSensor> analog_sensor_;

public:
    Controller(std::shared_ptr<Motor> motor,
               std::shared_ptr<AnalogSensor> analog_sensor):
        motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        osi::start_thread(&Controller::loop, this);
    }

    static void loop(void* instance_pointer)
    {
        ((Controller*)(instance_pointer))->loop();
    }

    void loop()
    {
        Timer<10> time_logger("controller", 1000);
        while(true)
        {
            double current_target =
                    2 * (analog_sensor_->get_latest_analogs() - 0.5);
            motor_->set_current_target(current_target);

            // print -----------------------------------------------------------
            Timer<>::sleep_ms(1);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                osi::print_to_screen("sending current: %f\n", current_target);
            }
        }
    }
};




int main(int argc, char **argv)
{  
    osi::initialize_realtime_printing();

    // create bus and boards -------------------------------------------------
    auto can_bus1 = std::make_shared<XenomaiCanbus>("rtcan0");
    auto can_bus2 = std::make_shared<XenomaiCanbus>("rtcan1");
    auto board1 = std::make_shared<XenomaiCanMotorboard>(can_bus1);
    auto board2 = std::make_shared<XenomaiCanMotorboard>(can_bus2);

    // create motors and sensors ---------------------------------------------
    auto motor_1 = std::make_shared<Motor>(board1, BLMC_MTR1);
    auto motor_2 = std::make_shared<Motor>(board1, BLMC_MTR2);
    auto motor_3 = std::make_shared<Motor>(board2, BLMC_MTR1);

    auto analog_sensor_1 = std::make_shared<AnalogSensor>(board1, BLMC_ADC_A);
    auto analog_sensor_2 = std::make_shared<AnalogSensor>(board1, BLMC_ADC_B);
    auto analog_sensor_3 = std::make_shared<AnalogSensor>(board2, BLMC_ADC_A);

    Controller controller1(motor_1, analog_sensor_1);
    Controller controller2(motor_2, analog_sensor_2);
    Controller controller3(motor_3, analog_sensor_3);

    // somehow this is necessary to be able to use some of the functionality
    osi::make_this_thread_realtime();
    board1->enable();
    board2->enable();

    controller1.start_loop();
    controller2.start_loop();
    controller3.start_loop();

    while(true)
    {
        Timer<>::sleep_ms(10);
    }

    return 0;
}
