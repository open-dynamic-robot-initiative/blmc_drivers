#pragma once

//#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>

#include <iostream>
#include <stdlib.h>
#include <memory>
#include <eigen3/Eigen/Core>
#include <array>
#include <tuple>
#include <string>
#include <map>

#include <time_logger.hpp>
#include <threadsafe_object.hpp>
#include <os_interface.hpp>


//long unsigned count_xenomai_mode_switches = 0;


//// TODO: make sure that mode switcheds are actually detected
//void action_upon_switch(int sig __attribute__((unused)))
//{
//  void *bt[32];
//  int nentries;

//  // increment mode swich counter
//  ++count_xenomai_mode_switches;

//  osi::print_to_screen("MOOOOODE SWIIIIITCH\n");
//  exit(-1);
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
    /// public interface =======================================================
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
        int ret = setup_can(&can_connection_old_format,
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
    SingletypeThreadsafeObject<CanConnection, 1> connection_info_;
    SingletypeThreadsafeObject<StampedFrame, 1> output_;
    SingletypeThreadsafeObject<StampedFrame, 1> input_;

    // methods -----------------------------------------------------------------
    static void
#ifndef __XENO__
    *
#endif
    loop(void* instance_pointer)
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



    int setup_can(CAN_CanHandle_t canHandle, char const *interface,
                  uint32_t err_mask)
    {
        CAN_CanConnection_t *can = (CAN_CanConnection_t*)canHandle;
        int ret;
        struct ifreq ifr;


        ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (ret < 0) {
            rt_fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
            return -1;
        }
        can->socket = ret;


        // Select interface
        if (interface == NULL) {
            ifr.ifr_ifindex = 0;
        } else {
            strncpy(ifr.ifr_name, interface, IFNAMSIZ);
            ret = rt_dev_ioctl(can->socket, SIOCGIFINDEX, &ifr);
            if (ret < 0) {
                rt_fprintf(stderr, "rt_dev_ioctl GET_IFINDEX: %s\n",
                           strerror(-ret));
                CAN_closeCan(canHandle);
                return -1;
            }
        }


        // Set error mask
        if (err_mask) {
            ret = rt_dev_setsockopt(can->socket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                    &err_mask, sizeof(err_mask));
            if (ret < 0) {
                rt_fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
                CAN_closeCan(canHandle);
                return -1;
            }
        }


        //if (filter_count) {
        //    ret = rt_dev_setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER,
        //                            &recv_filter, filter_count *
        //                            sizeof(struct can_filter));
        //    if (ret < 0) {
        //        rt_fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
        //        goto failure;
        //    }
        //}


        // Bind to socket
        can->recv_addr.can_family = AF_CAN;
        can->recv_addr.can_ifindex = ifr.ifr_ifindex;
        ret = rt_dev_bind(can->socket, (struct sockaddr *)&can->recv_addr,
                          sizeof(struct sockaddr_can));
        if (ret < 0) {
            rt_fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
            CAN_closeCan(canHandle);
            return -1;
        }

#ifdef __XENO__
        // Enable timestamps for frames
        ret = rt_dev_ioctl(can->socket,
                           RTCAN_RTIOC_TAKE_TIMESTAMP, RTCAN_TAKE_TIMESTAMPS);
        if (ret) {
            rt_fprintf(stderr, "rt_dev_ioctl TAKE_TIMESTAMP: %s\n",
                       strerror(-ret));
            CAN_closeCan(canHandle);
            return -1;
        }
#elif defined __RT_PREEMPT__
        // TODO: Need to support timestamps.
#endif


        can->recv_addr.can_family = AF_CAN;
        can->recv_addr.can_ifindex = ifr.ifr_ifindex;

        can->msg.msg_iov = &can->iov;
        can->msg.msg_iovlen = 1;
        can->msg.msg_name = (void *)&can->msg_addr;
        can->msg.msg_namelen = sizeof(struct sockaddr_can);
        can->msg.msg_control = (void *)&can->timestamp;
        can->msg.msg_controllen = sizeof(nanosecs_abs_t);

        // TODO why the memset?
        memset(&can->send_addr, 0, sizeof(can->send_addr));
        can->send_addr.can_family = AF_CAN;
        can->send_addr.can_ifindex = ifr.ifr_ifindex;


        return 0;
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




struct MotorboardStatus
{                             // bits
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
};




class MotorboardInterface
{
public:
    typedef StampedData<double> StampedScalar;
    typedef StampedData<MotorboardCommand> StampedCommand;
    typedef StampedData<MotorboardStatus> StampedStatus;

    /// outputs ================================================================
    std::map<std::string, size_t> measurement_map_ = {{"current_0", 0},
                                                      {"current_1", 1},
                                                      {"position_0", 2},
                                                      {"position_1", 3},
                                                      {"velocity_0", 4},
                                                      {"velocity_1", 5},
                                                      {"analog_0", 6},
                                                      {"analog_1", 7},
                                                      {"encoder_0", 8},
                                                      {"encoder_1", 9}};


    std::vector<std::string> measurement_names_ = {{"current_0",
                                                   "current_1",
                                                   "position_0",
                                                   "position_1",
                                                   "velocity_0",
                                                   "velocity_1",
                                                   "analog_0",
                                                   "analog_1",
                                                   "encoder_0",
                                                   "encoder_1"}};

    std::map<std::string, size_t> status_map_ = {{"status", 0}};

    /// inputs =================================================================
    std::map<std::string, size_t> control_map_ = {{"current_target_0", 0},
                                                  {"current_target_1", 1}};

    std::map<std::string, size_t> command_map_ = {{"command", 0}};

public:
    // get output data ---------------------------------------------------------
    virtual StampedScalar get_measurement(const std::string& name) const = 0;
    virtual void wait_for_measurement(const std::string& name) const = 0;
    virtual void wait_for_any_measurement() const = 0;

    virtual StampedStatus get_status(const std::string& name) const = 0;
    virtual void wait_for_status(const std::string& name) const = 0;
    virtual void wait_for_any_status() const = 0;

    // get input data ---------------------------------------------------------
    virtual StampedScalar get_control(const std::string& name) const = 0;
    virtual void wait_for_control(const std::string& name) const = 0;
    virtual void wait_for_any_control() const = 0;

    virtual StampedCommand get_command(const std::string& name) const = 0;
    virtual void wait_for_command(const std::string& name) const = 0;
    virtual void wait_for_any_command() const = 0;

    // send input data ---------------------------------------------------------
    virtual void send_control(const StampedScalar& control,
                              const std::string& name) = 0;
    virtual void send_command(const StampedCommand& command,
                              const std::string& name) = 0;

    virtual ~MotorboardInterface() {}
};










class XenomaiCanMotorboard: public MotorboardInterface
{
    /// public interface =======================================================
public:
    // get output data ---------------------------------------------------------
    virtual StampedScalar get_measurement(const std::string& name) const
    {
        return measurements_.get(name);
    }
    virtual void wait_for_measurement(const std::string& name) const
    {
        measurements_.wait_for_update(name);
    }
    virtual void wait_for_any_measurement() const
    {
        measurements_.wait_for_update();
    }

    virtual StampedStatus get_status(const std::string& name) const
    {
        return status_.get(status_map_.at(name));
    }
    virtual void wait_for_status(const std::string& name) const
    {
        status_.wait_for_update(status_map_.at(name));
    }
    virtual void wait_for_any_status() const
    {
        status_.wait_for_update();
    }

    // get input data ---------------------------------------------------------
    virtual StampedScalar get_control(const std::string& name) const
    {
        return controls_.get(control_map_.at(name));
    }
    virtual void wait_for_control(const std::string& name) const
    {
        controls_.wait_for_update(control_map_.at(name));
    }
    virtual void wait_for_any_control() const
    {
        controls_.wait_for_update();
    }

    virtual StampedCommand get_command(const std::string& name) const
    {
        return command_.get(command_map_.at(name));
    }
    virtual void wait_for_command(const std::string& name) const
    {
        command_.wait_for_update(command_map_.at(name));
    }
    virtual void wait_for_any_command() const
    {
        command_.wait_for_update();
    }

    // send input data ---------------------------------------------------------
    virtual void send_control(const StampedScalar& control,
                              const std::string& name)
    {
        controls_.set(control, control_map_.at(name));
        send_current_targets();
    }

    void send_command(const StampedCommand& command, const std::string& name)
    {
        command_.set(command, command_map_.at(name));

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
        can_frame.id = CanframeIDs::COMMAND_ID;
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
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                                      MotorboardCommand::Contents::ENABLE),-1,-1), "command");
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::SEND_ALL,
                                                      MotorboardCommand::Contents::ENABLE),-1,-1), "command");
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR1,
                                                      MotorboardCommand::Contents::ENABLE),-1,-1), "command");
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_MTR2,
                                                      MotorboardCommand::Contents::ENABLE),-1,-1), "command");
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::SET_CAN_RECV_TIMEOUT,
                                                      100),-1,-1), "command");
    }

    /// private members ========================================================
private:
    std::shared_ptr<XenomaiCanbus> can_bus_;

    // outputs -----------------------------------------------------------------
    SingletypeThreadsafeObject<StampedScalar, 10> measurements_;
    SingletypeThreadsafeObject<StampedStatus, 1> status_;

    // inputs ------------------------------------------------------------------
    SingletypeThreadsafeObject<StampedScalar, 2> controls_;
    SingletypeThreadsafeObject<StampedCommand, 1> command_;


    enum CanframeIDs
    {
        COMMAND_ID   = 0x00,
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
    XenomaiCanMotorboard(std::shared_ptr<XenomaiCanbus> can_bus):
        can_bus_(can_bus),
        measurements_(measurement_names_)
    {

        for(size_t i = 0; i < measurements_.size(); i++)
        {
            measurements_.set(StampedScalar(0, -1, -1), i);
        }
        measurements_.set(StampedScalar(0.5, -1, -1), "analog_0");
        measurements_.set(StampedScalar(0.5, -1, -1), "analog_1");

        for(size_t i = 0; i < controls_.size(); i++)
        {
            controls_.set(StampedScalar(0, -1, -1), i);
        }


        osi::start_thread(&XenomaiCanMotorboard::loop, this);
    }

    ~XenomaiCanMotorboard()
    {
        send_command(StampedCommand(MotorboardCommand(MotorboardCommand::IDs::ENABLE_SYS,
                                                      MotorboardCommand::Contents::DISABLE),-1,-1), "command");
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

    void send_current_targets()
    {
        Eigen::Vector2d current_targets;
        current_targets[0] = get_control("current_target_0").get_data();
        current_targets[1] = get_control("current_target_1").get_data();

        float current_mtr1 = current_targets[0];
        float current_mtr2 = current_targets[1];

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

        CanFrame can_frame;
        can_frame.id = BLMC_CAN_ID_IqRef;
        for(size_t i = 0; i < 7; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        return can_bus_->input_send_can_frame(StampedData<CanFrame>(can_frame, -1, -1));
    }

    static void
#ifndef __XENO__
    *
#endif
    loop(void* instance_pointer)
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
            double measurement_0 = qbytes_to_float(can_frame.data.begin());
            double measurement_1 =
                    qbytes_to_float((can_frame.data.begin() + 4));

            StampedScalar
                    stamped_measurement_0(measurement_0,
                                          stamped_can_frame.get_id(),
                                          stamped_can_frame.get_time_stamp());
            StampedScalar
                    stamped_measurement_1(measurement_1,
                                          stamped_can_frame.get_id(),
                                          stamped_can_frame.get_time_stamp());



            switch(can_frame.id)
            {
            case CanframeIDs::Iq:
                measurements_.set(stamped_measurement_0, "current_0");
                measurements_.set(stamped_measurement_1, "current_1");
                break;
            case CanframeIDs::POS:
                measurements_.set(stamped_measurement_0, "position_0");
                measurements_.set(stamped_measurement_1, "position_1");
                break;
            case CanframeIDs::SPEED:
                measurements_.set(stamped_measurement_0, "velocity_0");
                measurements_.set(stamped_measurement_1, "velocity_1");
                break;
            case CanframeIDs::ADC6:
                measurements_.set(stamped_measurement_0, "analog_0");
                measurements_.set(stamped_measurement_1, "analog_1");
                break;
            case CanframeIDs::ENC_INDEX:
            {
                // here the interpretation of the message is different,
                // we get a motor index and a measurement
                uint8_t motor_index = can_frame.data[4];
                StampedScalar measurement = stamped_measurement_0;
                if(motor_index == 0)
                {
                    measurements_.set(measurement, "encoder_0");
                }
                else if(motor_index == 1)
                {
                    measurements_.set(measurement, "encoder_1");
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

                StampedData<MotorboardStatus>
                        stamped_status(status,
                                       stamped_can_frame.get_id(),
                                       stamped_can_frame.get_time_stamp());

                status_.set(stamped_status, status_map_.at("status"));
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
        auto status = status_.get();
        osi::print_to_screen("status: time_stamp = %f, id = %d ---------------\n", status.get_time_stamp(), status.get_id());
        //        BLMC_printStatus(&status.get_data());

        rt_printf("\tSystem enabled: %d\n", status.get_data().system_enabled);
        rt_printf("\tMotor 1 enabled: %d\n", status.get_data().motor1_enabled);
        rt_printf("\tMotor 1 ready: %d\n", status.get_data().motor1_ready);
        rt_printf("\tMotor 2 enabled: %d\n", status.get_data().motor2_enabled);
        rt_printf("\tMotor 2 ready: %d\n", status.get_data().motor2_ready);
        rt_printf("\tError Code: %d\n", status.get_data().error_code);
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

class MotorInterface
{
public:
    typedef StampedData<double> StampedScalar;
    typedef StampedData<MotorboardCommand> StampedCommand;
    typedef StampedData<MotorboardStatus> StampedStatus;



public:
    /// measurements: current, position, velocity, encoder =====================
    virtual StampedScalar get_measurement(const std::string& name) const = 0;
    virtual void wait_for_measurement(const std::string& name) const = 0;
    virtual void wait_for_any_measurement() const = 0;


    /// controls: current_target ===============================================
    virtual StampedScalar get_control(const std::string& name) const = 0;
    virtual void wait_for_control(const std::string& name) const = 0;
    virtual void wait_for_any_control() const = 0;


    // send input data ---------------------------------------------------------
    virtual void send_control(const StampedScalar& control,
                              const std::string& name) = 0;

    virtual ~MotorInterface() {}
};
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class Motor
{
    std::map<std::string, std::string> measurement_map_;
    std::map<std::string, std::string> control_map_;

    std::shared_ptr<XenomaiCanMotorboard> board_;
    bool motor_id_;
public:

    Motor(std::shared_ptr<XenomaiCanMotorboard> board, unsigned motor_id):
        board_(board), motor_id_(motor_id)
    {
        if(motor_id == 0)
        {
            measurement_map_ = {{"current", "current_0"},
                                {"position", "position_0"},
                                {"velocity", "velocity_0"},
                                {"encoder", "encoder_0"}};

            control_map_ = {{"current_target", "current_target_0"}};
        }
        else
        {
            measurement_map_ = {{"current", "current_1"},
                                {"position", "position_1"},
                                {"velocity", "velocity_1"},
                                {"encoder", "encoder_1"}};

            control_map_ = {{"current_target", "current_target_1"}};
        }

    }


    double get_latest_currents()
    {
        if(motor_id_ == 0)
            return board_->get_measurement("current_0").get_data();
        else
            return board_->get_measurement("current_1").get_data();
    }
    double get_latest_positions()
    {
        if(motor_id_ == 0)
            return board_->get_measurement("position_0").get_data();
        else
            return board_->get_measurement("position_1").get_data();
    }
    double get_latest_velocities()
    {
        if(motor_id_ == 0)
            return board_->get_measurement("velocity_0").get_data();
        else
            return board_->get_measurement("velocity_1").get_data();
    }
    double get_latest_encoders()
    {
        if(motor_id_ == 0)
            return board_->get_measurement("encoder_0").get_data();
        else
            return board_->get_measurement("encoder_1").get_data();
    }

    void set_current_target(double current_target)
    {
        if(motor_id_ == 0)
            return board_->send_control(
                        StampedData<double>(current_target, -1, -1),
                        "current_target_0");
        else
            return board_->send_control(
                        StampedData<double>(current_target, -1, -1),
                        "current_target_1");    }
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
            return board_->get_measurement("analog_0").get_data();
        else
            return board_->get_measurement("analog_1").get_data();
    }
};

