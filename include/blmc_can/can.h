/**
 * \file
 * \brief API for CAN access. This is a wrapper for RTCAN.
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 *
 * \defgroup CanApi CAN API
 * \brief API for the CAN connection.
 *
 * This is basically a wrapper for Xenomais rtcan module for simple CAN
 * communication.  It is independent of the concrete application, i.e. it does
 * not contain anything related to the motor control board.
 *
 * \{
 */
#ifndef CAN_H_
#define CAN_H_

// INCLUDES
// **************************************************************************
#ifdef __XENO__
#include <rtdm/rtcan.h>
#elif defined __RT_PREEMPT__
// Include header for SocketCAN.
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

// Define typedefs to make code compatible with Xenoami code.
typedef struct can_frame can_frame_t;
typedef canid_t can_id_t;
typedef uint64_t 	nanosecs_abs_t;

#define rt_fprintf fprintf
#define rt_printf printf

#endif

#ifdef __cplusplus
extern "C"{
#endif


// TYPEDEFS
// **************************************************************************

//! \brief Encapsulates everything that is required to send/receive CAN frames.
typedef struct _CAN_CanConnection_t_
{
    struct sockaddr_can recv_addr;
    struct sockaddr_can send_addr;
    struct sockaddr_can msg_addr;
    can_frame_t recv_frame;
    can_frame_t send_frame;
    struct msghdr msg;
    struct iovec iov;
    nanosecs_abs_t timestamp;
    int socket;
} CAN_CanConnection_t;  // TODO better name

//! \brief Handle to the CAN connection object.
typedef CAN_CanConnection_t *CAN_CanHandle_t;


//! \brief Represents a CAN frame.
typedef struct _CAN_Frame_t_
{
    //! Data content of the frame. Up to 8 bytes (see dlc).
    uint8_t *data;
    //! Number of bytes in data.
    uint8_t dlc;
    //! Arbitration Id of the frame.
    can_id_t id;
    //! Timestamp of the moment the frame was received.
    nanosecs_abs_t timestamp;
    //! Index of the interface from which the frame was received.
    int recv_ifindex;
} CAN_Frame_t;


// FUNCTIONS
// **************************************************************************


//! \brief Initialize the CAN handle.
//! \param can_con Pointer to an CAN connection object.
//! \returns CAN handle that encapsulates the given connection object.
CAN_CanHandle_t CAN_initCanHandle(CAN_CanConnection_t *can_con);


//! \brief Set the CAN connection up
//!
//! This has to be done before sending or receiving messages.
//!
//! \param handle The CAN connection handle.
//! \param interface Name of the CAN interface. If NULL, the socket is
//!                  connected to all available interfaces.
//! \param err_mask Error mask. TODO what is this?
int CAN_setupCan(CAN_CanHandle_t canHandle, char const *interface,
        uint32_t err_mask);


//! \brief Close CAN connection
//! \param handle The CAN connection handle.
//! \returns Return value of rt_dev_close or 0 if there is no open connection.
int CAN_closeCan(CAN_CanHandle_t handle);


//! \brief Receive a frame via CAN
//!
//! \note This function is not thread-safe.  Do not call it in parallel with
//!       the same CanHandle.  Parallel execution with different handles should
//!       be fine, though.
//!
//! \param handle The CAN connection handle.
//! \param out_frame The data of the received frame will be written to this
//!                  structure.  NOTE: the data will only be valid until the
//!                  next call of this function!
int CAN_receiveFrame(CAN_CanHandle_t handle, CAN_Frame_t *out_frame);


//! \brief Send a data frame via CAN
//!
//! \note This function is not thread-safe.  Do not call it in parallel with
//!       the same CanHandle.  Parallel execution with different handles should
//!       be fine, though.
//!
//! \param handle The CAN connection handle.
//! \param id The CAN arbitration id of the frame.
//! \param data The data to be sent with the frame (max. 8 bytes!)
//! \param dlc The number of bytes stored in data (max. 8 bytes).
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int CAN_sendFrame(CAN_CanHandle_t handle, uint32_t id, uint8_t *data,
        uint8_t dlc);

#ifdef __cplusplus
}
#endif

/** \} */ // end group CanAPI

#endif  // CAN_H_
