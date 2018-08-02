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
    nanosecs_abs_t timestamp;
    // \todo: do we need this?
    int recv_ifindex;
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
public:
    StampedData()
    {
        id_ = 0;
        time_stamp_ = 0;
    }
    StampedData(const DataType& data, const size_t& id, const double& time_stamp)
    {
        data_ = data;
        id_ = id;
        time_stamp_ = time_stamp;
    }

    DataType get_data()
    {
        return data_;
    }

    /// \todo: rename to get_id
    size_t get_id()
    {
        return id_;
    }

    double get_time_stamp()
    {
        return time_stamp_;
    }


    void set_data(DataType data)
    {
        data_ = data;
    }


private:
    DataType data_;
    size_t id_;
    double time_stamp_;
};




RT_TASK start_thread(void (*function)(void *cookie), void *argument=NULL)
{
    // TODO: not sure if this is the right place for this
    mlockall(MCL_CURRENT | MCL_FUTURE);
    //        signal(SIGTERM, cleanup_and_exit);
    //        signal(SIGINT, cleanup_and_exit);
    //        signal(SIGDEBUG, action_upon_switch);
    rt_print_auto_init(1);

    RT_TASK rt_task;
    int priority = 10;

    int return_task_create = rt_task_create(&rt_task, NULL, 0, priority,  T_JOINABLE | T_FPU);
    if (return_task_create) {
        rt_fprintf(stderr, "controller: %s\n", strerror(-return_task_create));
        exit(-1);
    }
    rt_task_start(&rt_task, function, argument);

    return rt_task;
}




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


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class XenomaiCanBus
{
private:
    RT_TASK rt_task_;

    CanConnection can_connection_;
    RT_MUTEX can_connection_mutex_;

    typedef OldThreadsafeObject<StampedData<CanFrame>> Output;
    typedef OldThreadsafeObject<StampedData<CanFrame>> Input;


    Output output_;
    Input input_;


    // send and get ============================================================
public:

    // get output data ---------------------------------------------------------
    StampedData<CanFrame> output_get_can_frame()
    {
        return output_.get();
    }
    void output_wait_for_can_frame()
    {
        output_.wait_for_update();
    }
    size_t output_wait_for_any()
    {
        return output_.wait_for_update();
    }

    // get input data ----------------------------------------------------------
    StampedData<CanFrame> input_get_can_frame()
    {
        return input_.get();
    }
    void input_wait_for_can_frame()
    {
        input_.wait_for_update();
    }
    size_t input_wait_for_any()
    {
        return input_.wait_for_update();
    }

    // send data ---------------------------------------------------------------
    void send_can_frame(StampedData<CanFrame> stamped_can_frame)
    {
        input_.set(stamped_can_frame);

        // get address ---------------------------------------------------------
        rt_mutex_acquire(&can_connection_mutex_, TM_INFINITE);
        int socket = can_connection_.socket;
        struct sockaddr_can address = can_connection_.send_addr;
        rt_mutex_release(&can_connection_mutex_);


        auto unstamped_can_frame = stamped_can_frame.get_data();
        // put data into can frame ---------------------------------------------
        can_frame_t can_frame;
        can_frame.can_id = unstamped_can_frame.id;
        can_frame.can_dlc = unstamped_can_frame.dlc;


        memcpy(can_frame.data, unstamped_can_frame.data.begin(), unstamped_can_frame.dlc);

        // send ----------------------------------------------------------------
        int ret = rt_dev_sendto(socket,
                                (void *)&can_frame,
                                sizeof(can_frame_t),
                                0,
                                (struct sockaddr *)&address,
                                sizeof(address));
        if (ret < 0)
        {
            rt_printf("something went wrong with "
                      "sending CAN frame, error code: %d\n", ret);
            exit(-1);
        }
    }

    // constructor and destructor ==============================================
public: 
    XenomaiCanBus(std::string can_interface_name)
    {
        rt_mutex_create(&can_connection_mutex_, NULL);

        // setup can connection --------------------------------
        // \todo get rid of old format stuff
        CAN_CanConnection_t can_connection_old_format;
        int ret = CAN_setupCan(&can_connection_old_format,
                               can_interface_name.c_str(), 0);
        if (ret < 0)
        {
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }

        // \todo:how do we make sure that can connection is closed when we close
        //        can_connections.push_back(can_connection_old_format);
        can_connection_.send_addr = can_connection_old_format.send_addr;
        can_connection_.socket = can_connection_old_format.socket;


        rt_task_ = start_thread(&XenomaiCanBus::loop, this);
    }
    virtual ~XenomaiCanBus()
    {
        int ret = rt_dev_close(can_connection_.socket);
        if (ret)
        {
            rt_fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
            exit(-1);
        }
    }

    // private =================================================================
private:
    static void loop(void* instance_pointer)
    {
        ((XenomaiCanBus*)(instance_pointer))->loop();
    }

    void loop()
    {

        TimeLogger<100> loop_time_logger("can bus loop", 4000);
        TimeLogger<100> receive_time_logger("receive", 4000);

        while (true)
        {
            receive_time_logger.start_interval();
            CanFrame frame = receive_frame();
            receive_time_logger.end_interval();


            // to give the client time to read the latest message
            // we sleep for a microsecond --------------------------------------
            rt_task_sleep(1000);


            output_.set<0>(StampedData<CanFrame>(
                                  frame,
                                  output_.get<0>().get_id() + 1,
                                  TimeLogger<1>::current_time()));

            loop_time_logger.end_and_start_interval();
        }
    }

    CanFrame receive_frame()
    {
        rt_mutex_acquire(&can_connection_mutex_, TM_INFINITE);
        int socket = can_connection_.socket;
        rt_mutex_release(&can_connection_mutex_);

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
        int ret = rt_dev_recvmsg(socket, &message_header, 0);
        if (ret < 0)
        {
            rt_printf("something went wrong with receiving "
                      "CAN frame, error code: %d\n", ret);
            exit(-1);
        }

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
        out_frame.timestamp = timestamp;
        out_frame.recv_ifindex = message_address.can_ifindex;

        return out_frame;
    }
};






//class XenomaiCanMotorboardInput: public ThreadsafeObject
//{

//};

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


class TICanMotorBoard
{
    /// public interface =======================================================
public:
    class Command
    {
    public:

        Command()
        {

        }

        Command(uint32_t id, int32_t content)
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

    enum InputNames {
        CURRENT_TARGETS,
        COMMAND
    };

    typedef OldThreadsafeObject<
    StampedData<Eigen::Vector2d>,
    StampedData<Command>> Input;


    /// \todo: can we make this an enum class??
    enum OutputNames {
        CURRENTS,
        POSITIONS,
        VELOCITIES,
        ANALOGS,
        ENCODER0,
        ENCODER1,
        STATUS };

    typedef OldThreadsafeObject<
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<double>,
    StampedData<double>,
    StampedData<_BLMC_StatusMsg_t_> > Output;



    Input::Type<CURRENT_TARGETS> input_get_current_targets()
    {
        return input_.get<CURRENT_TARGETS>();
    }
    Input::Type<COMMAND> input_get_command()
    {
        return input_.get<COMMAND>();
    }

    void input_wait_for_current_targets()
    {
        input_.wait_for_update(CURRENT_TARGETS);
    }
    void input_wait_for_command()
    {
        input_.wait_for_update(COMMAND);
    }

    size_t input_wait_for_any()
    {
        return input_.wait_for_update();
    }


    Output::Type<CURRENTS> output_get_currents()
    {
        return output_.get<CURRENTS>();
    }
    Output::Type<POSITIONS> output_get_positions()
    {
        return output_.get<POSITIONS>();
    }
    Output::Type<VELOCITIES> output_get_velocities()
    {
        return output_.get<VELOCITIES>();
    }
    Output::Type<ANALOGS> output_get_analogs()
    {
        return output_.get<ANALOGS>();
    }
    Output::Type<ENCODER0> output_get_encoder_0()
    {
        return output_.get<ENCODER0>();
    }
    Output::Type<ENCODER1> output_get_encoder_1()
    {
        return output_.get<ENCODER1>();
    }
    Output::Type<STATUS> output_get_status()
    {
        return output_.get<STATUS>();
    }


    void output_wait_for_currents()
    {
        output_.wait_for_update(CURRENTS);
    }
    void output_wait_for_positions()
    {
        output_.wait_for_update(POSITIONS);
    }
    void output_wait_for_velocities()
    {
        output_.wait_for_update(VELOCITIES);
    }
    void output_wait_for_analogs()
    {
        output_.wait_for_update(ANALOGS);
    }
    void output_wait_for_encoder_0()
    {
        output_.wait_for_update(ENCODER0);
    }
    void output_wait_for_encoder_1()
    {
        output_.wait_for_update(ENCODER1);
    }
    void output_wait_for_status()
    {
        output_.wait_for_update(STATUS);
    }

    size_t output_wait_for_any()
    {
        return output_.wait_for_update();
    }





    void send_command(Command command)
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

        CanFrame can_frame;
        can_frame.id = BLMC_CAN_ID_COMMAND;
        for(size_t i = 0; i < 8; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->send_can_frame(StampedData<CanFrame>(can_frame, -1, -1));
    }

    void send_current_target(double current, unsigned motor_id)
    {
        rt_mutex_acquire(&current_targets_mutex_, TM_INFINITE);
        Eigen::Vector2d current_targets = current_targets_;
        rt_mutex_release(&current_targets_mutex_);

        current_targets[id_to_index(motor_id)] = current;
        send_current_targets(current_targets);
    }

    // todo: this should go away
    void enable()
    {
        send_command(Command(Command::IDs::ENABLE_SYS,
                             Command::Contents::ENABLE));
        send_command(Command(Command::IDs::SEND_ALL,
                             Command::Contents::ENABLE));
        send_command(Command(Command::IDs::ENABLE_MTR1,
                             Command::Contents::ENABLE));
        send_command(Command(Command::IDs::ENABLE_MTR2,
                             Command::Contents::ENABLE));
    }

    /// private members ========================================================
private:
    std::shared_ptr<XenomaiCanBus> can_bus_;


    RT_TASK rt_task_;

    Input input_;

    Output output_;



    BLMC_BoardData_t data_;
    RT_MUTEX data_mutex_;

    // controls
    Eigen::Vector2d current_targets_;
    RT_MUTEX current_targets_mutex_;

    /// constructor ============================================================
public:
    TICanMotorBoard(std::shared_ptr<XenomaiCanBus> can_bus): can_bus_(can_bus)
    {
        rt_mutex_create(&data_mutex_, NULL);
        rt_mutex_create(&current_targets_mutex_, NULL);

        // initialize members
        BLMC_initBoardData(&data_, BLMC_SYNC_ON_ADC6);
        current_targets_.setZero();

        // initialize members ------------------------------------------------------
        StampedData<Eigen::Vector2d>
                stamped_default_measurement(Eigen::Vector2d::Zero(), -1, -1);
        output_.set<CURRENTS>(stamped_default_measurement);
        output_.set<POSITIONS>(stamped_default_measurement);
        output_.set<VELOCITIES>(stamped_default_measurement);
        output_.set<ANALOGS>(StampedData<Eigen::Vector2d>(Eigen::Vector2d(0.5, 0.5), -1, -1));

        output_.set<ENCODER0>(StampedData<double>(0, -1, -1));
        output_.set<ENCODER1>(StampedData<double>(0, -1, -1));

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


        rt_task_ = start_thread(&TICanMotorBoard::loop, this);
    }

    /// private methods ========================================================
private:
    void send_current_targets(Eigen::Vector2d currents)
    {
        rt_mutex_acquire(&current_targets_mutex_, TM_INFINITE);
        current_targets_ = currents;
        rt_mutex_release(&current_targets_mutex_);

        float current_mtr1 = currents[0];
        float current_mtr2 = currents[1];

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

        return can_bus_->send_can_frame(StampedData<CanFrame>(can_frame, -1, -1));
    }

    static void loop(void* instance_pointer)
    {
        ((TICanMotorBoard*)(instance_pointer))->loop();
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
            measurement[0] = QBYTES_TO_FLOAT(can_frame.data.begin());
            measurement[1] = QBYTES_TO_FLOAT((can_frame.data.begin() + 4));

            StampedData<Eigen::Vector2d>
                    stamped_measurement(measurement,
                                        stamped_can_frame.get_id(),
                                        stamped_can_frame.get_time_stamp());

            switch(can_frame.id)
            {
            case BLMC_CAN_ID_Iq:
                output_.set<CURRENTS>(stamped_measurement);
                break;
            case BLMC_CAN_ID_POS:
                output_.set<POSITIONS>(stamped_measurement);
                break;
            case BLMC_CAN_ID_SPEED:
                output_.set<VELOCITIES>(stamped_measurement);
                break;
            case BLMC_CAN_ID_ADC6:
                output_.set<ANALOGS>(stamped_measurement);
                break;
            case BLMC_CAN_ID_ENC_INDEX:
            {
                uint8_t motor_index = can_frame.data[4];
                StampedData<double> stamped_encoder(
                            measurement[0],
                        stamped_can_frame.get_id(),
                        stamped_can_frame.get_time_stamp());
                if(motor_index == 0)
                {
                    output_.set<ENCODER0>(stamped_encoder);
                }
                else if(motor_index == 1)
                {
                    output_.set<ENCODER1>(stamped_encoder);
                }
                else
                {
                    rt_printf("ERROR: Invalid motor number"
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
                auto status = output_.get<STATUS>().get_data();
                BLMC_printStatus(&status);
                //                BLMC_printSensorData(&bd->latest);
                //                BLMC_printEncoderIndex(bd->encoder_index);


                //                BLMC_printLatestBoardStatus(&data_);
            }
            count++;

        }
    }

    unsigned id_to_index(unsigned motor_id)
    {
        if(motor_id == BLMC_MTR1)
            return 0;
        else if(motor_id == BLMC_MTR2)
            return 1;

        rt_printf("unknown motor id: %d", motor_id);
        exit(-1);
    }
};


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class Motor
{
    // \todo: should probably make this a shared pointer
    std::shared_ptr<TICanMotorBoard> board_;
    unsigned motor_id_;
public:

    Motor(std::shared_ptr<TICanMotorBoard> board, unsigned motor_id):
        board_(board), motor_id_(motor_id) { }

    double get_latest_currents()
    {
        return board_->output_get_currents().get_data()(motor_id_);
    }
    double get_latest_positions()
    {
        return board_->output_get_currents().get_data()(motor_id_);
    }
    double get_latest_velocities()
    {
        return board_->output_get_currents().get_data()(motor_id_);
    }
    double get_latest_encoders()
    {
        if(motor_id_ == 0)
        {
            return board_->output_get_encoder_0().get_data();
        }
        else
        {
            return board_->output_get_encoder_1().get_data();
        }
    }

    void set_current_target(double current_target)
    {
        board_->send_current_target(current_target, motor_id_);
    }
};




class AnalogSensor
{
    std::shared_ptr<TICanMotorBoard> board_;
    unsigned sensor_id_;
public:

    AnalogSensor(std::shared_ptr<TICanMotorBoard> board, unsigned sensor_id):
        board_(board), sensor_id_(sensor_id) { }

    double get_latest_analogs()
    {
        return board_->output_get_analogs().get_data()(sensor_id_);
    }
};



class Controller
{
private:
    RT_TASK rt_task_;

    // \todo: should probably make this a shared pointer
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<AnalogSensor> analog_sensor_;

public:
    Controller(std::shared_ptr<Motor> motor, std::shared_ptr<AnalogSensor> analog_sensor):
        motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        // for memory management
        mlockall(MCL_CURRENT | MCL_FUTURE);

        //        signal(SIGTERM, cleanup_and_exit);
        //        signal(SIGINT, cleanup_and_exit);

        // start real-time thread ------------------------------------------------------------------
        // for real time printing
        rt_print_auto_init(1);
        int priority = 10;
        int return_task_create = rt_task_create(&rt_task_, NULL, 0, priority,  T_JOINABLE | T_FPU);
        if (return_task_create) {
            rt_fprintf(stderr, "controller: %s\n", strerror(-return_task_create));
            exit(-1);
        }
        rt_task_start(&rt_task_, &Controller::loop, this);
    }


    static void loop(void* instance_pointer)
    {
        ((Controller*)(instance_pointer))->loop();
    }


    void loop()
    {
        TimeLogger<10> time_logger("controller", 1000);
        while(true)
        {
            double current_target = 2 * (analog_sensor_->get_latest_analogs() - 0.5);
            motor_->set_current_target(current_target);

            // print --------------------------------------------------------------
            rt_task_sleep(1000000);
            time_logger.end_and_start_interval();
            if ((time_logger.count() % 1000) == 0)
            {
                rt_printf("sending current: %f\n", current_target);
            }
        }
    }


    int join()
    {
        rt_task_join(&rt_task_);
    }
};




int main(int argc, char **argv)
{

    //    {
    //     Device<std::tuple<int>, std::tuple<float, int, int, double, bool>> device;
    //     device.show_types();
    //    }

    //    exit(-1);
    rt_print_auto_init(1);


    // create bus and boards -------------------------------------------------
    auto can_bus1 = std::make_shared<XenomaiCanBus>("rtcan0");
    auto can_bus2 = std::make_shared<XenomaiCanBus>("rtcan1");
    auto board1 = std::make_shared<TICanMotorBoard>(can_bus1);
    auto board2 = std::make_shared<TICanMotorBoard>(can_bus2);

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



    rt_task_shadow(NULL, "shibuya", 0, 0);
    board1->enable();
    board2->enable();

    controller1.start_loop();
    controller2.start_loop();
    controller3.start_loop();

    while(true)
    {
        rt_task_sleep(1000000);
    }

    return 0;
}
