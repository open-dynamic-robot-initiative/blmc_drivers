/**
 * @file can_bus.cpp
 * @author Felix Widmaier (felix.widmaier@tuebingen.mpg.de)
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file defines classes that allow communication with a Can network.
 * @version 0.1
 * @date 2018-11-23
 *
 * @copyright Copyright (c) 2018
 *
 */

#include <sstream>

#include <blmc_drivers/devices/can_bus.hpp>

namespace blmc_drivers
{
CanBus::CanBus(const std::string &can_interface_name,
               const size_t &history_length)
{
    input_ = std::make_shared<CanframeTimeseries>(history_length, 0, false);
    sent_input_ =
        std::make_shared<CanframeTimeseries>(history_length, 0, false);
    output_ = std::make_shared<CanframeTimeseries>(history_length, 0, false);
    name_ = can_interface_name;

    can_connection_.set(setup_can(can_interface_name, 0));

    is_loop_active_ = true;
    rt_thread_.create_realtime_thread(&CanBus::loop, this);
}

CanBus::~CanBus()
{
    is_loop_active_ = false;
    rt_thread_.join();
    osi::close_can_device(can_connection_.get().socket);
}

void CanBus::send_if_input_changed()
{
    if (input_->has_changed_since_tag())
    {
        time_series::Index timeindex_to_send = input_->newest_timeindex();
        CanBusFrame frame_to_send = (*input_)[timeindex_to_send];
        input_->tag(timeindex_to_send);
        sent_input_->append(frame_to_send);

        send_frame(frame_to_send);
    }
}

void CanBus::loop()
{
    while (is_loop_active_)
    {
        output_->append(receive_frame());
    }
}

void CanBus::send_frame(const CanBusFrame &unstamped_can_frame)
{
    // get address ---------------------------------------------------------
    int socket = can_connection_.get().socket;
    struct sockaddr_can address = can_connection_.get().send_addr;

    // put data into can frame ---------------------------------------------
    // make sure to initialise the whole struct to zero to avoid issues when
    // using a CAN-FD-capable device.
    can_frame_t can_frame = {};
    can_frame.can_id = unstamped_can_frame.id;
    can_frame.can_dlc = unstamped_can_frame.dlc;

    memcpy(can_frame.data,
           unstamped_can_frame.data.begin(),
           unstamped_can_frame.dlc);

    // send ----------------------------------------------------------------
    osi::send_to_can_device(socket,
                            (void *)&can_frame,
                            sizeof(can_frame_t),
                            0,
                            (struct sockaddr *)&address,
                            sizeof(address));
}

CanBusFrame CanBus::receive_frame()
{
    int socket = can_connection_.get().socket;

    // data we want to obtain ----------------------------------------------
    can_frame_t can_frame = {};
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
    for (size_t i = 0; i < can_frame.can_dlc; i++)
    {
        out_frame.data[i] = can_frame.data[i];
    }

    return out_frame;
}

CanBusConnection CanBus::setup_can(std::string name, uint32_t err_mask)
{
    int socket_number;
    sockaddr_can recv_addr;
    sockaddr_can send_addr;
    struct ifreq ifr;

    int ret;

    ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (ret < 0)
    {
        rt_fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
        rt_printf("Couldn't setup CAN connection. Exit.");
        exit(-1);
    }
    socket_number = ret;

    strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ);
    ret = rt_dev_ioctl(socket_number, SIOCGIFINDEX, &ifr);
    if (ret < 0)
    {
        rt_fprintf(stderr, "rt_dev_ioctl GET_IFINDEX: %s\n", strerror(-ret));
        osi::close_can_device(socket_number);
        rt_printf("Couldn't setup CAN connection. Exit.");
        exit(-1);
    }

    // Set error mask
    if (err_mask)
    {
        ret = rt_dev_setsockopt(socket_number,
                                SOL_CAN_RAW,
                                CAN_RAW_ERR_FILTER,
                                &err_mask,
                                sizeof(err_mask));
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
    ret = rt_dev_bind(socket_number,
                      (struct sockaddr *)&recv_addr,
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
    ret =
        rt_dev_ioctl(socket, RTCAN_RTIOC_TAKE_TIMESTAMP, RTCAN_TAKE_TIMESTAMPS);
    if (ret)
    {
        rt_fprintf(stderr, "rt_dev_ioctl TAKE_TIMESTAMP: %s\n", strerror(-ret));
        osi::close_can_device(socket);
        rt_printf("Couldn't setup CAN connection. Exit.");
        exit(-1);
    }
#elif defined RT_PREEMPT
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

}  // namespace blmc_drivers
