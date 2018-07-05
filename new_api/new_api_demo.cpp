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

#include <rtdk.h>
#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>
#include <blmc_can/optoforce_can.h>
#include <functional>


#include <iostream>
#include <stdlib.h>

#include <eigen3/Eigen/Core>


#include <array>
#include <time_logger.hpp>




long unsigned count_xenomai_mode_switches = 0;


// TODO: make sure that mode switcheds are actually detected
void action_upon_switch(int sig __attribute__((unused)))
{
  void *bt[32];
  int nentries;

  // increment mode swich counter
  ++count_xenomai_mode_switches;

  rt_printf("MOOOOODE SWIIIIITCH\n");
  exit(-1);
}

// GLOBALS
// **************************************************************************
std::vector<CAN_CanConnection_t> can_connections;


// FUNCTIONS
// **************************************************************************
void cleanup_and_exit(int sig)
{
    rt_printf("Signal %d received\n", sig);
    // Disable system before closing connection

    for(int i = 0; i < can_connections.size(); i++)
    {
        BLMC_sendCommand(&can_connections[i], BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
        CAN_closeCan(&can_connections[i]);
    }
    exit(0);
}


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


#define FLOAT_TO_Q24(fval) ((int)(fval * (1 << 24)))

class CanBus
{
public:
    typedef std::function<void(CanFrame)> Callback;

private:
    CanConnection can_connection_;
    RT_MUTEX can_connection_mutex_;

    std::vector<Callback> callbacks_;
    unsigned callback_count_;
    RT_MUTEX callback_mutex_;

    RT_TASK rt_task_;
    bool rt_task_is_running_;
    RT_MUTEX rt_task_mutex_;

public: 
    CanBus(std::string can_interface_name, unsigned max_callback_count = 10)
    {
        rt_mutex_create(&can_connection_mutex_, NULL);
        rt_mutex_create(&callback_mutex_, NULL);
        rt_mutex_create(&rt_task_mutex_, NULL);

        // setup can connection ----------------------------------------------------------
        // \todo get rid of old format stuff
        CAN_CanConnection_t can_connection_old_format;
        int ret = CAN_setupCan(&can_connection_old_format, can_interface_name.c_str(), 0);
        if (ret < 0) {
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }
        can_connections.push_back(can_connection_old_format);
        can_connection_.send_addr = can_connection_old_format.send_addr;
        can_connection_.socket = can_connection_old_format.socket;

        callbacks_.resize(max_callback_count);
        callback_count_ = 0;
        rt_task_is_running_ = false;

        // TODO: not sure if this is the right place for this
        mlockall(MCL_CURRENT | MCL_FUTURE);
        signal(SIGTERM, cleanup_and_exit);
        signal(SIGINT, cleanup_and_exit);
        signal(SIGDEBUG, action_upon_switch);
        rt_print_auto_init(1);


        start_loop();
    }

    void add_callback(Callback callback)
    {
        rt_mutex_acquire(&callback_mutex_, TM_INFINITE);

        if(callback_count_ >= callbacks_.size())
        {
            rt_printf("you exceeded the max number of callbacks\n");
            exit(-1);
        }

        callbacks_[callback_count_] = callback;
        callback_count_++;

        rt_mutex_release(&callback_mutex_);
    }

    void loop()
    {
        TimeLogger<100> loop_time_logger("can bus loop", 4000);
        TimeLogger<100> receive_time_logger("receive", 4000);


        while (true) {
            receive_time_logger.start_interval();
            CanFrame frame = receive_frame();
            receive_time_logger.end_interval();

            rt_mutex_acquire(&callback_mutex_, TM_INFINITE);
            for(int i = 0; i < callback_count_; i++)
            {
                callbacks_[i](frame);
            }
            rt_mutex_release(&callback_mutex_);

            loop_time_logger.end_and_start_interval();
        }
    }

    static void close_can(int socket)
    {
        int ret = rt_dev_close(socket);
        if (ret)
        {
            rt_fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
            exit(-1);
        }
    }


    static void loop(void* instance_pointer)
    {
        ((CanBus*)(instance_pointer))->loop();
    }

    void start_loop()
    {
        rt_mutex_acquire(&rt_task_mutex_, TM_INFINITE);

        if(rt_task_is_running_)
        {
            return;
        }

        int priority = 10;
        int return_task_create = rt_task_create(&rt_task_, NULL, 0, priority,  T_JOINABLE | T_FPU);
        if (return_task_create) {
            rt_fprintf(stderr, "rt_task_shadow: %s\n", strerror(-return_task_create));
            close_can(can_connection_.socket);
            exit(-1);
        }

        rt_task_start(&rt_task_, &CanBus::loop, this);
        rt_task_is_running_ = true;

        rt_mutex_release(&rt_task_mutex_);
    }

    int join()
    {
        rt_mutex_acquire(&rt_task_mutex_, TM_INFINITE);
        rt_task_join(&rt_task_);
        rt_mutex_release(&rt_task_mutex_);
    }

    void send_frame(uint32_t id, uint8_t *data, uint8_t dlc)
    {
        // get address ----------------------------------------
        rt_mutex_acquire(&can_connection_mutex_, TM_INFINITE);
        int socket = can_connection_.socket;
        struct sockaddr_can address = can_connection_.send_addr;
        rt_mutex_release(&can_connection_mutex_);

        // put data into can frame ----------------------------
        can_frame_t can_frame;
        can_frame.can_id = id;
        can_frame.can_dlc = dlc;
        memcpy(can_frame.data, data, dlc);

        // send ----------------------------------------------------
        int ret = rt_dev_sendto(socket, (void *)&can_frame,
                sizeof(can_frame_t), 0, (struct sockaddr *)&address,
                sizeof(address));
        if (ret < 0)
        {
            rt_printf("something went wrong with sending CAN frame, error code: %d\n", ret);
            exit(-1);
        }
    }

private:
    static CAN_CanConnection_t setup_can(std::string can_interface_name)
    {
        CAN_CanConnection_t can_connection;

        int return_can_setup = CAN_setupCan(&can_connection, can_interface_name.c_str(), 0);
        if (return_can_setup < 0) {
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }

        return can_connection;
    }

    CanFrame receive_frame()
    {
        rt_mutex_acquire(&can_connection_mutex_, TM_INFINITE);
        int socket = can_connection_.socket;
        rt_mutex_release(&can_connection_mutex_);

        // data we want to obtain ------------------------------------------------------
        can_frame_t can_frame;
        nanosecs_abs_t timestamp;
        struct sockaddr_can message_address;

        // setup message such that data can be received to variables above -------------
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

        // receive message from can bus ------------------------------------------------
        int ret = rt_dev_recvmsg(socket, &message_header, 0);
        if (ret < 0)
        {
            rt_printf("something went wrong with receiving CAN frame, error code: %d\n", ret);
            exit(-1);
        }


        // process received data and put into felix widmaier's format --------------------
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


class Board
{
    // \todo: add time stamps!
private:
    // should probably make this a shared pointer
    CanBus& can_bus_;

    BLMC_BoardData_t data_;
    RT_MUTEX data_mutex_;

    // controls
    Eigen::Vector2d current_targets_;
    RT_MUTEX current_targets_mutex_;


public:
    Board(CanBus& can_bus): can_bus_(can_bus)
    {
        rt_mutex_create(&data_mutex_, NULL);
        rt_mutex_create(&current_targets_mutex_, NULL);

        // register callback for obtaining data from can bus
        CanBus::Callback callback = std::bind(&Board::consume_can_frame, this, std::placeholders::_1);
        can_bus_.add_callback(callback);

        // initialize members
        BLMC_initBoardData(&data_, BLMC_SYNC_ON_ADC6);
        current_targets_.setZero();
    }

    void enable()
    {
        send_command(BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);
        send_command(BLMC_CMD_SEND_ALL, BLMC_ENABLE);
        send_command(BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
        send_command(BLMC_CMD_ENABLE_MTR2, BLMC_ENABLE);
    }

    void consume_can_frame(CanFrame frame)
    {
        // \todo: this conversion is necessary now because the BLMC_processCanFrame
        // function has not been updated yet to use new frame format.
        CAN_Frame_t frame_old_format;
        frame_old_format.data = frame.data.begin();
        frame_old_format.dlc = frame.dlc;
        frame_old_format.id = frame.id;
        frame_old_format.timestamp = frame.timestamp;
        frame_old_format.recv_ifindex = frame.recv_ifindex;

        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        BLMC_processCanFrame(&frame_old_format, &data_);

        static int count = 0;
        if(count % 4000 == 0)
            BLMC_printLatestBoardStatus(&data_);
        count++;

        rt_mutex_release(&data_mutex_);
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

        can_bus_.send_frame(BLMC_CAN_ID_COMMAND, data, 8);
    }

    void set_current_targets(Eigen::Vector2d currents)
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

        return can_bus_.send_frame(BLMC_CAN_ID_IqRef, data, 8);
    }
    void set_current_target(double current, unsigned motor_id)
    {
        rt_mutex_acquire(&current_targets_mutex_, TM_INFINITE);
        Eigen::Vector2d current_targets = current_targets_;
        rt_mutex_release(&current_targets_mutex_);

        current_targets[id_to_index(motor_id)] = current;
        set_current_targets(current_targets);
    }

    double get_current_measurement(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double current_measurement = data_.latest.current.value[id_to_index(motor_id)];
        rt_mutex_release(&data_mutex_);

        return current_measurement;
    }
    double get_position_measurement(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double position_measurement = data_.latest.position.value[id_to_index(motor_id)];
        rt_mutex_release(&data_mutex_);

        return position_measurement;
    }
    double get_velocity_measurement(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double velocity_measurement = data_.latest.velocity.value[id_to_index(motor_id)];
        rt_mutex_release(&data_mutex_);

        return velocity_measurement;
    }
    double get_encoder_measurement(unsigned motor_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double encoder_measurement = data_.encoder_index[id_to_index(motor_id)].value;
        rt_mutex_release(&data_mutex_);

        return encoder_measurement;
    }
    double get_analog_measurement(unsigned adc_id)
    {
        rt_mutex_acquire(&data_mutex_, TM_INFINITE);
        double analog_measurement = data_.latest.adc6.value[id_to_index(adc_id)];
        rt_mutex_release(&data_mutex_);

        return analog_measurement;
    }

private:
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



class Motor
{
    // \todo: should probably make this a shared pointer
    Board& board_;
    unsigned motor_id_;
public:

    Motor(Board& board, unsigned motor_id): board_(board), motor_id_(motor_id) { }

    double get_current_measurement()
    {
        return board_.get_current_measurement(motor_id_);
    }
    double get_position_measurement()
    {
        return board_.get_position_measurement(motor_id_);
    }
    double get_velocity_measurement()
    {
        return board_.get_velocity_measurement(motor_id_);
    }
    double get_encoder_measurement()
    {
        return board_.get_encoder_measurement(motor_id_);
    }

    void set_current_target(double current_target)
    {
        board_.set_current_target(current_target, motor_id_);
    }
};


//class Finger
//{
//    std::array<Motor&, 3> motors_;
//};



class AnalogSensor
{
    Board& board_;
    unsigned sensor_id_;
public:

    AnalogSensor(Board& board, unsigned sensor_id): board_(board), sensor_id_(sensor_id) { }

    double get_analog_measurement()
    {
        board_.get_analog_measurement(sensor_id_);
    }
};



class Controller
{
private:
    RT_TASK rt_task_;

    // \todo: should probably make this a shared pointer
    Motor& motor_;
    AnalogSensor& analog_sensor_;

public:
    Controller(Motor& motor, AnalogSensor& analog_sensor): motor_(motor), analog_sensor_(analog_sensor) { }

    void start_loop()
    {
        // for memory management
        mlockall(MCL_CURRENT | MCL_FUTURE);

        signal(SIGTERM, cleanup_and_exit);
        signal(SIGINT, cleanup_and_exit);

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
            double current_target = 2 * (analog_sensor_.get_analog_measurement() - 0.5);
            motor_.set_current_target(current_target);

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
    CanBus can_bus1("rtcan0");
    CanBus can_bus2("rtcan1");

    Board board1(can_bus1);
    Board board2(can_bus2);

    Motor motor_1(board1, BLMC_MTR1);
    Motor motor_2(board1, BLMC_MTR2);
    Motor motor_3(board2, BLMC_MTR1);



    AnalogSensor analog_sensor_1(board1, BLMC_ADC_A);
    AnalogSensor analog_sensor_2(board1, BLMC_ADC_B);
    AnalogSensor analog_sensor_3(board2, BLMC_ADC_A);




    Controller controller1(motor_1, analog_sensor_1);
    Controller controller2(motor_2, analog_sensor_2);
    Controller controller3(motor_3, analog_sensor_3);



    rt_task_shadow(NULL, "shibuya", 0, 0);

    board1.enable();
    board2.enable();

    controller1.start_loop();
    controller2.start_loop();
    controller3.start_loop();




//    can_bus1.join();
//    can_bus2.join();

    while(true)
    {
        rt_task_sleep(1000000);
    }


    return 0;
}
