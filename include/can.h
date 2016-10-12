/**
 * API for CAN access. This is a wrapper for RTCAN.
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 */
#ifndef CAN_H_
#define CAN_H_

// INCLUDES
// **************************************************************************
#include <rtdm/rtcan.h>


// TYPEDEFS
// **************************************************************************

//! \brief Encapsulates everything that is required to send/receive CAN frames.
typedef struct _CAN_CanConnection_t_
{
    struct sockaddr_can recv_addr;
    struct sockaddr_can send_addr;
    struct sockaddr_can msg_addr;
    can_frame_t frame;
    struct msghdr msg;
    struct iovec iov;
    nanosecs_abs_t timestamp;
    int socket;
} CAN_CanConnection_t;  // TODO better name

//! \brief Handle to the CAN connection object.
typedef CAN_CanConnection_t *CAN_CanHandle_t;


typedef struct _CAN_Frame_t_
{
    uint8_t *data;
    uint8_t dlc;
    nanosecs_abs_t timestamp;
    int recv_ifindex;
} CAN_Frame_t;


// FUNCTIONS
// **************************************************************************


//! \brief Initialize the CAN handle.
//! \param can_con Pointer to an CAN connection object.
//! \returns CAN handle that encapsulates the given connection object.
inline CAN_CanHandle_t CAN_initCanHandle(CAN_CanConnection_t *can_con);


//! \brief Set the CAN connection up
//!
//! This has to be done before sending or receiving messages.
//!
//! \param handle The CAN connection handle.
//! \param interface Name of the CAN interface. If NULL, the socket is
//!                  connected to all available interfaces.
//! \param err_mask Error mask. TODO what is this?
int CAN_setupCan(CAN_CanHandle_t canHandle, char* interface,
        uint32_t err_mask);


//! \brief Close CAN connection
//! \param handle The CAN connection handle.
//! \returns Return value of rt_dev_close or 0 if there is no open connection.
int CAN_closeCan(CAN_CanHandle_t handle);


//! \brief Receive a frame via CAN
//!
//! \param handle The CAN connection handle.
//! \param out_frame The data of the received frame will be written to this
//!                  structure.  NOTE: the data will only be valid until the
//!                  next call of this function!
int CAN_receiveFrame(CAN_CanHandle_t handle, CAN_Frame_t *out_frame);


//! \brief Send a data frame via CAN
//!
//! \param handle The CAN connection handle.
//! \param id The CAN arbitration id of the frame.
//! \param data The data to be sent with the frame (max. 8 bytes!)
//! \param dlc The number of bytes stored in data (max. 8 bytes).
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int CAN_sendFrame(CAN_CanHandle_t handle, uint32_t id, uint8_t *data,
        uint8_t dlc);

#endif  // CAN_H_
