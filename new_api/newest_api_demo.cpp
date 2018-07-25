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





//template <typename...>
//struct Device;

//template <typename...InputTypes, typename...OutputTypes>
//class Device< std::tuple<InputTypes...>, std::tuple<OutputTypes...> >
//{
//public:
//    template<int I> using InputType = typename std::tuple_element<I, std::tuple<InputTypes...>>::type;
//    template<int I> using OutputType = typename std::tuple_element<I, std::tuple<OutputTypes...>>::type;

//    static const std::size_t input_size = sizeof...(InputTypes);
//    static const std::size_t output_size = sizeof...(OutputTypes);

//    typedef std::tuple<StampedData<InputTypes> ...> InputTuple;
//    typedef std::tuple<StampedData<OutputTypes> ...> OutputTuple;

//private:
//    OutputTuple outputs_;

//    std::array<RT_COND, output_size> output_conditions_;
//    RT_COND output_global_condition_;

//    std::array<RT_MUTEX, output_size> output_mutexes_;
//    RT_MUTEX output_global_mutex_;

//public:
//    Device()
//    {
//        rt_mutex_create(&output_global_mutex_, NULL);
//        rt_cond_create(&output_global_condition_, NULL);

//        for(size_t i = 0; i < output_size; i++)
//        {
//            rt_mutex_create(&output_mutexes_[i], NULL);
//            rt_cond_create(&output_conditions_[i], NULL);
//        }
//    }

//    unsigned wait_for_output()
//    {
//        rt_mutex_acquire(&output_mutex_, TM_INFINITE);
//        size_t latest_count = output_.get_id();

//        while(output_.get_id() == latest_count)
//        {
//            rt_cond_wait(&output_condition_, &output_mutex_, TM_INFINITE);
//        }

//        rt_mutex_release(&output_mutex_);

//        return 0;
//    }

//    StampedData<CanFrame> get_latest_output()
//    {
//        rt_mutex_acquire(&output_mutex_, TM_INFINITE);
//        StampedData<CanFrame> output = output_;
//        rt_mutex_release(&output_mutex_);

//        return output;
//    }




//    virtual void set_input(InputTuple input_tuple) = 0;


//    OutputTuple get_latest_output()
//    {
//        return output_;
//    }

//    template<int I> OutputType<I> get_latest_output()
//    {
//        return output_[I];
//    }

//    template<int I> void wait_for_output()
//    {
//        // return when next data of type index is received
//        return;
//    }

//    unsigned wait_for_output()
//    {
//        // return when next data of type index is received
//        return;
//    }

//    static void show_types()
//    {
//     std::cout << __PRETTY_FUNCTION__ << std::endl;
//    }


//protected:
//    template<int I> void set_latest_output(OutputType<I> output)
//    {

//        rt_mutex_acquire(&output_mutexes_[I], TM_INFINITE);
//        rt_mutex_acquire(&output_global_mutex_, TM_INFINITE);
//        outputs_[I] = output;
//        rt_cond_broadcast(&output_conditions_[I]);
//        rt_cond_broadcast(&output_global_condition_);

//        rt_mutex_release(&output_global_mutex_);
//        rt_mutex_release(&output_mutexes_[I]);

//        output_conditions_

//        rt_cond_broadcast(&output_global_condition_);

//    }

//};








class XenomaiDevice
{
protected:
    RT_TASK rt_task_;

public:
    XenomaiDevice()
    {
        // TODO: not sure if this is the right place for this
        mlockall(MCL_CURRENT | MCL_FUTURE);
        //        signal(SIGTERM, cleanup_and_exit);
        //        signal(SIGINT, cleanup_and_exit);
        //        signal(SIGDEBUG, action_upon_switch);
        rt_print_auto_init(1);

        int priority = 10;
        int return_task_create = rt_task_create(&rt_task_, NULL, 0,
                                                priority,  T_JOINABLE | T_FPU);
        if (return_task_create) {
            rt_fprintf(stderr, "rt_task_shadow: %s\n",
                       strerror(-return_task_create));
            exit(-1);
        }

        rt_task_start(&rt_task_, &XenomaiDevice::loop, this);
    }


private:
    static void loop(void* instance_pointer)
    {
        ((XenomaiDevice*)(instance_pointer))->loop();
    }

    virtual void loop() = 0;

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


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class CanBus: public XenomaiDevice
{
private:
    CanConnection can_connection_;
    RT_MUTEX can_connection_mutex_;

    ThreadsafeObject<StampedData<CanFrame>> can_frame_;

    // send and get ============================================================
public:
    StampedData<CanFrame> get_latest_can_frame()
    {
        return can_frame_.get<0>();
    }
    void wait_for_can_frame_()
    {
        can_frame_.wait_for_datum(0);
    }
    unsigned wait_for_any_output()
    {
        return can_frame_.wait_for_datum();
    }

    void send_can_frame(uint32_t id, uint8_t *data, uint8_t dlc)
    {
        // get address ---------------------------------------------------------
        rt_mutex_acquire(&can_connection_mutex_, TM_INFINITE);
        int socket = can_connection_.socket;
        struct sockaddr_can address = can_connection_.send_addr;
        rt_mutex_release(&can_connection_mutex_);

        // put data into can frame ---------------------------------------------
        can_frame_t can_frame;
        can_frame.can_id = id;
        can_frame.can_dlc = dlc;
        memcpy(can_frame.data, data, dlc);

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
    CanBus(std::string can_interface_name)
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
    }
    virtual ~CanBus()
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


            can_frame_.set<0>(StampedData<CanFrame>(
                                  frame,
                                  can_frame_.get<0>().get_id() + 1,
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



/// Design Pattern >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class DevicePattern
{
    typedef int Torque;
    typedef int Speed;
    typedef int Distance;
    typedef int Current;

    // inputs ---------------------------------
    void send_torque(Torque torque)
    {

    }
    void send_speed(Speed speed)
    {

    }

    // outputs --------------------------------
    Distance get_latest_distance()
    {

    }
    void wait_for_distance()
    {

    }

    Current get_latest_current()
    {

    }
    void wait_for_current()
    {

    }

    void wait_for_any_output()
    {

    }
};


/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


class Board: public XenomaiDevice
{
    /// public interface =======================================================
public:
    /// \todo: can we make this an enum class??
    enum OutputNames {
        CURRENTS,
        POSITIONS,
        VELOCITIES,
        ANALOGS,
        ENCODER0,
        ENCODER1,
        STATUS };

    typedef ThreadsafeObject<
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<Eigen::Vector2d>,
    StampedData<double>,
    StampedData<double>,
    StampedData<_BLMC_StatusMsg_t_> > Output;

    const Output& output()
    {
        return output_;
    }

    double get_latest_currents(unsigned motor_id)
    {
        return output_.get<CURRENTS>().get_data()[motor_id];
    }
    double get_latest_positions(unsigned motor_id)
    {
        return output_.get<POSITIONS>().get_data()[motor_id];
    }
    double get_latest_velocities(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double velocities = data_.latest.velocity.value[id_to_index(motor_id)];
        rt_mutex_release(&data_mutex_);

        return velocities;
    }
    double get_latest_encoders(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double encoders = data_.encoder_index[id_to_index(motor_id)].value;
        rt_mutex_release(&data_mutex_);

        return encoders;
    }
    double get_latest_analogs(unsigned adc_id)
    {
        return output_.get<ANALOGS>().get_data()[adc_id];
    }

    void send_command(uint32_t cmd_id, int32_t value)
    {
        uint8_t data[8];

        // value
        data[0] = (value >> 24) & 0xFF;
        data[1] = (value >> 16) & 0xFF;
        data[2] = (value >> 8) & 0xFF;
        data[3] = value & 0xFF;

        // command
        data[4] = (cmd_id >> 24) & 0xFF;
        data[5] = (cmd_id >> 16) & 0xFF;
        data[6] = (cmd_id >> 8) & 0xFF;
        data[7] = cmd_id & 0xFF;

        can_bus_->send_can_frame(BLMC_CAN_ID_COMMAND, data, 8);
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
        send_command(BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);
        send_command(BLMC_CMD_SEND_ALL, BLMC_ENABLE);
        send_command(BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
        send_command(BLMC_CMD_ENABLE_MTR2, BLMC_ENABLE);
    }

    /// private members ========================================================
private:
    std::shared_ptr<CanBus> can_bus_;




    Output output_;



    BLMC_BoardData_t data_;
    RT_MUTEX data_mutex_;

    // controls
    Eigen::Vector2d current_targets_;
    RT_MUTEX current_targets_mutex_;

    /// constructor ============================================================
public:
    Board(std::shared_ptr<CanBus> can_bus): can_bus_(can_bus)
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
        output_.set<ANALOGS>(stamped_default_measurement);

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

        return can_bus_->send_can_frame(BLMC_CAN_ID_IqRef, data, 8);
    }


    void loop()
    {
        while(true)
        {
            // get latest can frame --------------------------------------------

            can_bus_->wait_for_any_output();
            auto stamped_can_frame = can_bus_->get_latest_can_frame();
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
    std::shared_ptr<Board> board_;
    unsigned motor_id_;
public:

    Motor(std::shared_ptr<Board> board, unsigned motor_id):
        board_(board), motor_id_(motor_id) { }

    double get_latest_currents()
    {
        return board_->get_latest_currents(motor_id_);
    }
    double get_latest_positions()
    {
        return board_->get_latest_positions(motor_id_);
    }
    double get_latest_velocities()
    {
        return board_->get_latest_velocities(motor_id_);
    }
    double get_latest_encoders()
    {
        return board_->get_latest_encoders(motor_id_);
    }

    void set_current_target(double current_target)
    {
        board_->send_current_target(current_target, motor_id_);
    }
};




class AnalogSensor
{
    std::shared_ptr<Board> board_;
    unsigned sensor_id_;
public:

    AnalogSensor(std::shared_ptr<Board> board, unsigned sensor_id):
        board_(board), sensor_id_(sensor_id) { }

    double get_latest_analogs()
    {
        board_->get_latest_analogs(sensor_id_);
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


    // create bus and boards -------------------------------------------------
    auto can_bus1 = std::make_shared<CanBus>("rtcan0");
    auto can_bus2 = std::make_shared<CanBus>("rtcan1");
    auto board1 = std::make_shared<Board>(can_bus1);
    auto board2 = std::make_shared<Board>(can_bus2);

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
