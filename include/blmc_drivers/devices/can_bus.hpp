#pragma once

#include <memory>
#include <string>

#include <blmc_drivers/utils/timer.hpp>
#include <blmc_drivers/utils/threadsafe_object.hpp>
#include <blmc_drivers/utils/threadsafe_timeseries.hpp>

#include <blmc_drivers/utils/os_interface.hpp>
#include <blmc_drivers/devices/device_interface.hpp>




class CanBusFrame
{
public:
    std::array<uint8_t, 8> data;
    uint8_t dlc;
    can_id_t id;
};

class CanBusConnection
{
public:
    struct sockaddr_can send_addr;
    int socket;
};

class CanBusInterface: public DeviceInterface
{
public:
    typedef ThreadsafeTimeseries<CanBusFrame> CanframeTimeseries;

    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    virtual std::shared_ptr<const CanframeTimeseries>
    get_output_frame() const = 0;

    // input logs --------------------------------------------------------------
    virtual std::shared_ptr<const CanframeTimeseries> get_input_frame() = 0;
    virtual std::shared_ptr<const CanframeTimeseries> get_sent_input_frame() = 0;

    /// setters ================================================================
    virtual void set_input_frame(const CanBusFrame& input_frame) = 0;

    /// sender =================================================================
    virtual void send_if_input_changed() = 0;

    /// ========================================================================

    virtual ~CanBusInterface() {}
};

class CanBus: public CanBusInterface
{
public:
    /// getters ================================================================
    // device outputs ----------------------------------------------------------
    std::shared_ptr<const CanframeTimeseries> get_output_frame() const
    {
        return output_;
    }
    // input logs --------------------------------------------------------------
    virtual std::shared_ptr<const CanframeTimeseries>  get_input_frame()
    {
        return input_;
    }
    virtual std::shared_ptr<const CanframeTimeseries> get_sent_input_frame()
    {
        return sent_input_;
    }

    /// setters ================================================================
    virtual void set_input_frame(const CanBusFrame& input_frame)
    {
        input_->append(input_frame);
    }

    /// sender =================================================================
    virtual void send_if_input_changed()
    {
        if(input_->has_changed_since_tag())
        {
            CanframeTimeseries::Index
                    timeindex_to_send = input_->newest_timeindex();
            CanBusFrame frame_to_send = (*input_)[timeindex_to_send];
            input_->tag(timeindex_to_send);
            sent_input_->append(frame_to_send);

            send_frame(frame_to_send);
        }
    }
    /// ========================================================================


    CanBus(const std::string& can_interface_name,
           const size_t& history_length = 1000)
    {
        input_ = std::make_shared<CanframeTimeseries>(history_length);
        sent_input_ = std::make_shared<CanframeTimeseries>(history_length);
        output_ = std::make_shared<CanframeTimeseries>(history_length);

        can_connection_.set(setup_can(can_interface_name, 0));

        osi::start_thread(&CanBus::loop, this);
    }

    virtual ~CanBus()
    {
        osi::close_can_device(can_connection_.get().socket);
    }

    /// private attributes and methods =========================================
private:
    // attributes --------------------------------------------------------------
    SingletypeThreadsafeObject<CanBusConnection, 1> can_connection_;

    std::shared_ptr<ThreadsafeTimeseries<CanBusFrame>> input_;
    std::shared_ptr<ThreadsafeTimeseries<CanBusFrame>> sent_input_;

    std::shared_ptr<ThreadsafeTimeseries<CanBusFrame>> output_;

    // methods -----------------------------------------------------------------
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((CanBus*)(instance_pointer))->loop();
    }

    void loop()
    {
        Timer<100> loop_time_logger("can bus loop");

        while (true)
        {
            output_->append(receive_frame());
            loop_time_logger.end_and_start_interval();
        }
    }

    // send input data ---------------------------------------------------------
    void send_frame(const CanBusFrame& unstamped_can_frame)
    {
        // get address ---------------------------------------------------------
        int socket = can_connection_.get().socket;
        struct sockaddr_can address = can_connection_.get().send_addr;

        // put data into can frame ---------------------------------------------
        can_frame_t can_frame;
        can_frame.can_id = unstamped_can_frame.id;
        can_frame.can_dlc = unstamped_can_frame.dlc;

        memcpy(can_frame.data, unstamped_can_frame.data.begin(),
               unstamped_can_frame.dlc);

        // send ----------------------------------------------------------------
        osi::send_to_can_device(socket,
                                (void *)&can_frame,
                                sizeof(can_frame_t),
                                0,
                                (struct sockaddr *)&address,
                                sizeof(address));
    }


    CanBusFrame receive_frame()
    {
        int socket = can_connection_.get().socket;

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

        CanBusFrame out_frame;
        out_frame.id = can_frame.can_id;
        out_frame.dlc = can_frame.can_dlc;
        for(size_t i = 0; i < can_frame.can_dlc; i++)
        {
            out_frame.data[i] = can_frame.data[i];
        }

        return out_frame;
    }

    CanBusConnection setup_can(std::string name, uint32_t err_mask)
    {
        int socket_number;
        sockaddr_can recv_addr;
        sockaddr_can send_addr;
        struct ifreq ifr;

        int ret;

        ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (ret < 0) {
            rt_fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }
        socket_number = ret;

        strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ);
        ret = rt_dev_ioctl(socket_number, SIOCGIFINDEX, &ifr);
        if (ret < 0)
        {
            rt_fprintf(stderr, "rt_dev_ioctl GET_IFINDEX: %s\n",
                       strerror(-ret));
            osi::close_can_device(socket_number);
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }

        // Set error mask
        if (err_mask) {
            ret = rt_dev_setsockopt(socket_number, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                    &err_mask, sizeof(err_mask));
            if (ret < 0)
            {
                rt_fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
                osi::close_can_device(socket_number);
                rt_printf("Couldn't setup CAN connection. Exit.");
                exit(-1);
            }
        }

        // Bind to socket
        recv_addr.can_family = AF_CAN;
        recv_addr.can_ifindex = ifr.ifr_ifindex;
        ret = rt_dev_bind(socket_number, (struct sockaddr *)&recv_addr,
                          sizeof(struct sockaddr_can));
        if (ret < 0)
        {
            rt_fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
            osi::close_can_device(socket_number);
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }

#ifdef __XENO__
        // Enable timestamps for frames
        ret = rt_dev_ioctl(socket,
                           RTCAN_RTIOC_TAKE_TIMESTAMP, RTCAN_TAKE_TIMESTAMPS);
        if (ret) {
            rt_fprintf(stderr, "rt_dev_ioctl TAKE_TIMESTAMP: %s\n",
                       strerror(-ret));
            osi::close_can_device(socket);
            rt_printf("Couldn't setup CAN connection. Exit.");
            exit(-1);
        }
#elif defined __RT_PREEMPT__
        // TODO: Need to support timestamps.
#endif

        // TODO why the memset?
        memset(&send_addr, 0, sizeof(send_addr));
        send_addr.can_family = AF_CAN;
        send_addr.can_ifindex = ifr.ifr_ifindex;

        CanBusConnection can_connection;
        can_connection.send_addr = send_addr;
        can_connection.socket = socket_number;

        return can_connection;
    }
};
