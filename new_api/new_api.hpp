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

#include <threadsafe_timeseries.hpp>


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

    void print_header() const
    {
        osi::print_to_screen("id: %d, time_stamp: %f\n", id_, time_stamp_);
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
    typedef ThreadsafeTimeseriesInterface<CanFrame> CanframeTimeseries;

    virtual std::shared_ptr<CanframeTimeseries> input() = 0;
    virtual std::shared_ptr<const CanframeTimeseries> output() const = 0;

    virtual void send_if_input_changed() = 0;

    virtual ~CanbusInterface() {}
};



class XenomaiCanbus: public CanbusInterface
{



    /// public interface =======================================================
public:
    virtual std::shared_ptr<CanframeTimeseries>  input()
    {
        return input_;
    }

    std::shared_ptr<const CanframeTimeseries> output() const
    {
        return output_;
    }

    virtual void send_if_input_changed()
    {
        long int new_hash = input_->next_timeindex();
        if(new_hash != input_hash_.get())
        {
            send_frame(input_->current_element());
            input_hash_.set(new_hash);
        }
    }

    // constructor and destructor ----------------------------------------------
    XenomaiCanbus(std::string can_interface_name)
    {
        input_ = std::make_shared<ThreadsafeTimeseries<CanFrame>>(1000);
        output_ = std::make_shared<ThreadsafeTimeseries<CanFrame>>(1000);
        input_hash_.set(input_->next_timeindex());


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

    std::shared_ptr<ThreadsafeTimeseriesInterface<CanFrame>> input_;
    SingletypeThreadsafeObject<long int, 1> input_hash_;
    std::shared_ptr<ThreadsafeTimeseriesInterface<CanFrame>> output_;







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

            output_->append(frame);

//            old_output_.set<0>(StampedData<CanFrame>(
//                               frame,
//                               old_output_.get<0>().get_id() + 1,
//                               Timer<1>::current_time_ms()));

            // sometimes can messages come in a burst, so we add a little wait
//            Timer<>::sleep_ms(0.1);

            loop_time_logger.end_and_start_interval();
        }
    }

    // send input data ---------------------------------------------------------
    void send_frame(const CanFrame& unstamped_can_frame)
    {
        // get address ---------------------------------------------------------
        int socket = connection_info_.get().socket;
        struct sockaddr_can address = connection_info_.get().send_addr;

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






//class CanMotorboardInput: public ThreadsafeObject
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






class MotorboardInterface
{
public:
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;
    typedef ThreadsafeTimeseriesInterface<MotorboardStatus> StatusTimeseries;
    typedef ThreadsafeTimeseriesInterface<MotorboardCommand> CommandTimeseries;

    /// outputs ================================================================
    std::vector<std::string> measurement_names_ = {"current_0",
                                                   "current_1",
                                                   "position_0",
                                                   "position_1",
                                                   "velocity_0",
                                                   "velocity_1",
                                                   "analog_0",
                                                   "analog_1",
                                                   "encoder_0",
                                                   "encoder_1"};
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;
    virtual std::shared_ptr<const StatusTimeseries> status() const = 0;

    /// inputs =================================================================
    std::vector<std::string> control_names_ = {"current_target_0",
                                               "current_target_1"};
    virtual std::shared_ptr<ScalarTimeseries> control(std::string name) = 0;
    virtual std::shared_ptr<CommandTimeseries> command() = 0;

    virtual void send_if_input_changed() = 0;

    /// ========================================================================
    virtual void print_status() = 0;
    virtual ~MotorboardInterface() {}
};


class CanMotorboard: public  MotorboardInterface
{
    /// public interface =======================================================
public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        return measurements_.at(name);
    }
    virtual std::shared_ptr<const StatusTimeseries> status() const
    {
        return status_;
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> control(std::string name)
    {
        return controls_.at(name);
    }
    virtual std::shared_ptr<CommandTimeseries> command()
    {
        return command_;
    }
    virtual void send_if_input_changed()
    {
        // initialize outputs --------------------------------------------------
        bool controls_have_changed = false;
        for(size_t i = 0; i < control_names_.size(); i++)
        {
            long int new_hash =
                    controls_.at(control_names_[i])->next_timeindex();
            if(new_hash != control_hashes_.get(control_names_[i]))
            {
                control_hashes_.set(new_hash, control_names_[i]);
                controls_have_changed = true;
            }
        }
        if(controls_have_changed)
        {
            send_controls();
        }

        long int new_hash = command_->next_timeindex();
        if(new_hash != command_hash_.get())
        {
            command_hash_.set(new_hash);
            send_command();
        }

    }

    /// ========================================================================
    void send_command(const MotorboardCommand& command)
    {
        command_->append(command);
        send_if_input_changed();
    }


    void send_command()
    {
//        old_command_.set(command, name);

        MotorboardCommand command = command_->current_element();

//        command_->append(command);

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
        can_frame.id = CanframeIDs::COMMAND_ID;
        for(size_t i = 0; i < 8; i++)
        {
            can_frame.data[i] = data[i];
        }
        can_frame.dlc = 8;

        can_bus_->input()->append(can_frame);
        can_bus_->send_if_input_changed();
    }

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

    /// private members ========================================================
private:
    std::shared_ptr<XenomaiCanbus> can_bus_;

    // outputs -----------------------------------------------------------------
    std::map<std::string, std::shared_ptr<ScalarTimeseries>> measurements_;
    std::shared_ptr<StatusTimeseries> status_;

    // inputs ------------------------------------------------------------------
    std::map<std::string, std::shared_ptr<ScalarTimeseries>> controls_;
    SingletypeThreadsafeObject<long int, 2> control_hashes_;
    std::shared_ptr<CommandTimeseries> command_;
    SingletypeThreadsafeObject<long int, 1> command_hash_;

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
    CanMotorboard(std::shared_ptr<XenomaiCanbus> can_bus):
        can_bus_(can_bus),
        control_hashes_(control_names_)
    {
        // initialize outputs --------------------------------------------------
        for(size_t i = 0; i < measurement_names_.size(); i++)
        {
            measurements_[measurement_names_[i]]
                    = std::make_shared<ThreadsafeTimeseries<double>>(1000);
        }
        status_ =
                std::make_shared<ThreadsafeTimeseries<MotorboardStatus>>(1000);

        // initialize outputs --------------------------------------------------
        for(size_t i = 0; i < control_names_.size(); i++)
        {
            controls_[control_names_[i]]
                    = std::make_shared<ThreadsafeTimeseries<double>>(1000);

            controls_.at(control_names_[i])->append(0);

            control_hashes_.set(
                        controls_.at(control_names_[i])->next_timeindex(),
                    control_names_[i]);
        }
        command_ =
                std::make_shared<ThreadsafeTimeseries<MotorboardCommand>>(1000);
        command_hash_.set(command_->next_timeindex());

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
        float current_mtr1 = controls_["current_target_0"]->current_element();
        float current_mtr2 = controls_["current_target_1"]->current_element();

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

        can_bus_->input()->append(can_frame);
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

        long int timeindex = can_bus_->output()->next_timeindex();
        while(true)
        {

//            osi::print_to_screen("waiting for can frame with index %d\n", timeindex);
            CanFrame can_frame = (*can_bus_->output())[timeindex];
            timeindex++;
//            osi::print_to_screen("received\n");

            // convert to measurement ------------------------------------------
            double measurement_0 = qbytes_to_float(can_frame.data.begin());
            double measurement_1 = qbytes_to_float((can_frame.data.begin() + 4));


            switch(can_frame.id)
            {
            case CanframeIDs::Iq:
                measurements_.at("current_0")->append(measurement_0);
                measurements_.at("current_1")->append(measurement_1);
                break;
            case CanframeIDs::POS:
                measurements_.at("position_0")->append(measurement_0);
                measurements_.at("position_1")->append(measurement_1);
                break;
            case CanframeIDs::SPEED:
                measurements_.at("velocity_0")->append(measurement_0);
                measurements_.at("velocity_1")->append(measurement_1);
                break;
            case CanframeIDs::ADC6:
                measurements_.at("analog_0")->append(measurement_0);
                measurements_.at("analog_1")->append(measurement_1);
                break;
            case CanframeIDs::ENC_INDEX:
            {
                // here the interpretation of the message is different,
                // we get a motor index and a measurement
                uint8_t motor_index = can_frame.data[4];
                if(motor_index == 0)
                {
                    measurements_.at("encoder_0")->append(measurement_0);
                }
                else if(motor_index == 1)
                {
                    measurements_.at("encoder_1")->append(measurement_0);
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
//            print_status();

        }
    }

    void print_status()
    {
        osi::print_to_screen("outputs =====================================\n");

        for(size_t i = 0; i < measurement_names_.size(); i++)
        {
            osi::print_to_screen("%s: ---------------------------------\n",
                                 measurement_names_[i].c_str());
            if(measurements_.at(measurement_names_[i])->size() > 0)
            {
                double measurement =
                        measurements_.at(measurement_names_[i])->current_element();
                osi::print_to_screen("value %f:\n", measurement);
            }
        }

        osi::print_to_screen("status: ---------------------------------\n");
        if(status_->size() > 0)
            status_->current_element().print();

        osi::print_to_screen("inputs ======================================\n");

        for(size_t i = 0; i < control_names_.size(); i++)
        {
            osi::print_to_screen("%s: ---------------------------------\n",
                                 control_names_[i].c_str());
            if(controls_.at(control_names_[i])->size() > 0)
            {
                double control =
                        controls_.at(control_names_[i])->current_element();
                osi::print_to_screen("value %f:\n", control);
            }
        }

        osi::print_to_screen("command: ---------------------------------\n");
        if(command_->size() > 0)
            command_->current_element().print();
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
class MotorInterface
{
public:
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;

    /// outputs ================================================================
    std::vector<std::string> measurement_names_ = {"current",
                                                   "position",
                                                   "velocity",
                                                   "encoder"};

    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;

    /// inputs =================================================================    
    virtual std::shared_ptr<ScalarTimeseries> current_target() = 0;
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~MotorInterface() {}
};




class Motor: public MotorInterface
{
    std::map<std::string, std::string> motor_to_board_name_;
    std::shared_ptr<CanMotorboard> board_;

public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        return board_->measurement(motor_to_board_name_.at(name));
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> current_target()
    {
        return board_->control(motor_to_board_name_.at("current_target"));
    }

    virtual void send_if_input_changed()
    {
        board_->send_if_input_changed();
    }

    /// ========================================================================

    Motor(std::shared_ptr<CanMotorboard> board, bool motor_id):
        board_(board)
    {
        for(size_t i = 0; i < measurement_names_.size(); i++)
        {
            motor_to_board_name_[measurement_names_[i]] =
                    measurement_names_[i] + "_" + std::to_string(motor_id);
        }
        motor_to_board_name_["current_target"] =
                "current_target_" + std::to_string(motor_id);
    }
};



//class SafeMotor: public Motor
//{
//public:
//    SafeMotor(std::shared_ptr<CanMotorboard> board, bool motor_id):
//        Motor(board, motor_id)
//    {
//        temperature_.set(StampedScalar(room_temperature_));
//        osi::start_thread(&SafeMotor::loop, this);
//    }


//    // send input data ---------------------------------------------------------
//    virtual void send_control(const StampedScalar& control,
//                              const std::string& name)
//    {
//    }

//private:
//    ThreadsafeObject<StampedScalar> temperature_;

//    const double room_temperature_ = 30;

//    static void
//#ifndef __XENO__
//    *
//#endif
//    loop(void* instance_pointer)
//    {
//        ((SafeMotor*)(instance_pointer))->loop();
//    }

//    void loop()
//    {
//        Timer<10> time_logger("controller", 1000);
//        while(true)
//        {
//            wait_for_measurement("current");
//            StampedScalar current = get_measurement("current");




//            // print -----------------------------------------------------------
//            Timer<>::sleep_ms(1);
//            time_logger.end_and_start_interval();
//            if ((time_logger.count() % 1000) == 0)
//            {
//            }
//        }
//    }
//};



class AnalogsensorInterface
{
public:
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const = 0;

    virtual ~AnalogsensorInterface() {}
};






class Analogsensor: public AnalogsensorInterface
{
    std::map<std::string, std::string> sensor_to_board_name_;
    std::string name_;
    std::shared_ptr<CanMotorboard> board_;

public:
    Analogsensor(std::shared_ptr<CanMotorboard> board, bool sensor_id):
        board_(board)
    {
        name_ = "analog_" + std::to_string(sensor_id);
    }

    virtual std::shared_ptr<const ScalarTimeseries> measurement() const
    {
        return board_->measurement(name_);
    }

};



///// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
class FingerInterface
{
public:
    typedef ThreadsafeTimeseriesInterface<double> ScalarTimeseries;

    /// outputs ================================================================
    std::vector<std::string> measurement_names_ = {"current_interior",
                                                   "current_center",
                                                   "current_tip",
                                                   "position_interior",
                                                   "position_center",
                                                   "position_tip"
                                                   "velocity_interior",
                                                   "velocity_center",
                                                   "velocity_tip",
                                                   "encoder_interior",
                                                   "encoder_center",
                                                   "encoder_tip"};

    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const = 0;

    /// inputs =================================================================
    std::vector<std::string> control_names_ = {"current_target_interior",
                                               "current_target_center",
                                               "current_target_tip"};

    virtual std::shared_ptr<ScalarTimeseries> control(std::string name) = 0;
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~FingerInterface() {}
};



class Finger: public FingerInterface
{
    std::map<std::string, std::shared_ptr<MotorInterface>> motors_;

public:
    /// outputs ================================================================
    virtual std::shared_ptr<const ScalarTimeseries>
    measurement(std::string name) const
    {
        std::string motor_name, content_name;
        parse_name(name, motor_name, content_name);

        return motors_.at(motor_name)->measurement(content_name);
    }

    /// inputs =================================================================
    virtual std::shared_ptr<ScalarTimeseries> control(std::string name)
    {
        std::string motor_name, content_name;
        parse_name(name, motor_name, content_name);

        return motors_.at(motor_name)->current_target();
    }

    virtual void send_if_input_changed()
    {
        motors_["interior"]->send_if_input_changed();
        motors_["center"]->send_if_input_changed();
        motors_["tip"]->send_if_input_changed();
    }

    /// ========================================================================

    Finger(std::shared_ptr<MotorInterface> interior_motor,
           std::shared_ptr<MotorInterface> center_motor,
           std::shared_ptr<MotorInterface> tip_motor)
    {
        motors_["interior"] = interior_motor;
        motors_["center"] = center_motor;
        motors_["tip"] = tip_motor;
    }

private:
    void parse_name(const std::string& name,
                    std::string& motor_name, std::string& content_name) const
    {
        size_t _position = name.rfind("_");
        content_name = name.substr(0, _position);
        motor_name = name;
        motor_name.erase(0, _position + 1);
    }
};


