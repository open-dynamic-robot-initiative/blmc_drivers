#include <blmc_can/can.h>
#include <string.h>

#ifdef __XENO__
#include <rtdk.h>
#else
// Defining the Xenomai (CAN) API for the Linux SocketCan via #define.
#define rt_dev_socket socket
#define rt_dev_ioctl ioctl
#define rt_dev_close close
#define rt_dev_setsockopt setsockopt
#define rt_dev_bind bind
#define rt_dev_recvmsg recvmsg
#define rt_dev_sendto sendto

#endif

// FUNCTIONS
// **************************************************************************

CAN_CanHandle_t CAN_initCanHandle(CAN_CanConnection_t *can_con)
{
    return (CAN_CanHandle_t) can_con;
}

int CAN_setupCan(CAN_CanHandle_t canHandle, char const *interface,
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
#else
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

int CAN_closeCan(CAN_CanHandle_t handle)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    int ret;

    if (can->socket >= 0) {
        ret = rt_dev_close(can->socket);
        can->socket = -1;
        if (ret) {
            rt_fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
        }
        return ret;
    } else {
        return 0;
    }
}

int CAN_receiveFrame(CAN_CanHandle_t handle, CAN_Frame_t *out_frame)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    int ret;

    can->iov.iov_base = (void *)&can->recv_frame;
    can->iov.iov_len = sizeof(can_frame_t);

    ret = rt_dev_recvmsg(can->socket, &can->msg, 0);

    if (can->msg.msg_controllen == 0) {
        // No timestamp for this frame available. Make sure we dont get
        // garbage.
        can->timestamp = 0;
    }

    if (ret >= 0) {
        out_frame->id = can->recv_frame.can_id;
        out_frame->data = can->recv_frame.data;
        out_frame->dlc = can->recv_frame.can_dlc;
#ifdef __XENO__
        out_frame->timestamp = can->timestamp;
#else
        // TODO: Need to support timestamps.
        // HACK: Adding a non-zero value here as part of the code (e.g. BLMC_printEncoderIndex)
        //       checks if the timestamp is nonzero before printing anything.
        out_frame->timestamp = 1;
#endif
        out_frame->recv_ifindex = can->msg_addr.can_ifindex;
    }

    return ret;
}

int CAN_sendFrame(CAN_CanHandle_t handle, uint32_t id, uint8_t *data,
        uint8_t dlc)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    int ret;

    can->send_frame.can_id = id;
    can->send_frame.can_dlc = dlc;
    memcpy(can->send_frame.data, data, dlc);

    ret = rt_dev_sendto(can->socket, (void *)&can->send_frame,
            sizeof(can_frame_t), 0, (struct sockaddr *)&can->send_addr,
            sizeof(can->send_addr));
    return ret;
}
