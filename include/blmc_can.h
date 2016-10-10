/**
 * API for the CAN interface of the Motor Control Microcontroller Boards.
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 */
#ifndef BLMC_CAN_H_
#define BLMC_CAN_H_

// INCLUDES
// **************************************************************************
#include <rtdm/rtcan.h>


// DEFINES
// **************************************************************************

// Arbitration IDs of the different message types
#define BLMC_CAN_ID_COMMAND    0x00
#define BLMC_CAN_ID_IqRef      0x05
#define BLMC_CAN_ID_STATUSMSG  0x10
#define BLMC_CAN_ID_Iq         0x20
#define BLMC_CAN_ID_POS        0x30
#define BLMC_CAN_ID_SPEED      0x40
#define BLMC_CAN_ID_ADC6       0x50


// COMMAND IDs
#define BLMC_CMD_ENABLE_SYS 1
#define BLMC_CMD_ENABLE_MTR1 2
#define BLMC_CMD_ENABLE_MTR2 3
#define BLMC_CMD_ENABLE_VSPRING1 4
#define BLMC_CMD_ENABLE_VSPRING2 5
#define BLMC_CMD_SEND_CURRENT 12
#define BLMC_CMD_SEND_POSITION 13
#define BLMC_CMD_SEND_VELOCITY 14
#define BLMC_CMD_SEND_ADC6 15
#define BLMC_CMD_SEND_ALL 20

#define BLMC_ENABLE 1
#define BLMC_DISABLE 0

// Motor Indices
#define BLMC_MTR1 0
#define BLMC_MTR2 1

// ADC Indices
#define BLMC_ADC_A 0
#define BLMC_ADC_B 1


// Macros
// ------

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


// TYPEDEFS
// **************************************************************************

// just for convenience
typedef struct can_frame frame_t;


//! \brief Encapsulates everything that is required to send/receive CAN frames.
typedef struct _BLMC_CanConnection_t_
{
    struct sockaddr_can recv_addr;
    struct sockaddr_can send_addr;
    struct sockaddr_can msg_addr;
    frame_t frame;
    struct msghdr msg;
    struct iovec iov;
    nanosecs_abs_t timestamp;
    int socket;
} BLMC_CanConnection_t;

//! \brief Handle to the CAN connection object.
typedef BLMC_CanConnection_t *BLMC_CanHandle_t;


//! \brief Board Status Message.
typedef struct _BLMC_StatusMsg_t_
{                             // bits
   uint8_t system_enabled:1;  // 0
   uint8_t motor1_enabled:1;  // 1
   uint8_t motor1_ready:1;    // 2
   uint8_t motor2_enabled:1;  // 3
   uint8_t motor2_ready:1;    // 4
   uint8_t error_code:3;      // 5-7
} BLMC_StatusMsg_t;


//! \brief A dual float value with timestamp
//!
//! This can be used to store a message from the board that contains two
//! related values (e.g. position of motor 1 and of motor 2) together with the
//! timestamp.
typedef struct _BLMC_StampedValue_t_  // TODO better name? (float and [2])
{
    //! Timestamp of the moment, the frame was received.
    nanosecs_abs_t timestamp;
    //! Pair of values (e.g. for the two motors).
    float value[2];
} BLMC_StampedValue_t;


//! \brief Bundles all data send by the board.
typedef struct _BLMC_BoardData_t_
{
    BLMC_StatusMsg_t status;
    BLMC_StampedValue_t current;
    BLMC_StampedValue_t position;
    BLMC_StampedValue_t velocity;
    BLMC_StampedValue_t adc6;
} BLMC_BoardData_t;


// FUNCTIONS
// **************************************************************************

//! \brief Initialize the CAN handle.
//! \param can_con Pointer to an CAN connection object.
//! \returns CAN handle that encapsulates the given connection object.
inline BLMC_CanHandle_t BLMC_initCanHandle(BLMC_CanConnection_t *can_con);


//! \brief Set the CAN connection up.
//!
//! This has to be done before sending or receiving messages.
//! \param handle The CAN connection handle.
// TODO more params
int BLMC_setupCan(BLMC_CanHandle_t canHandle, char* interface, uint32_t
        err_mask);


//! \brief Close CAN connection
//! \returns Return value of rt_dev_close or 0 if there is no open connection.
int BLMC_closeCan(BLMC_CanHandle_t handle);


//! \brief Initialize a stamped value to zero.
void BLMC_initStampedValue(BLMC_StampedValue_t *sv);


//! \brief Initialize status message (set everything to zero).
void BLMC_initStatus(BLMC_StatusMsg_t *st);


//! \brief Initialize board data structure.
void BLMC_initBoardData(BLMC_BoardData_t *bd);


//! \brief Decode a dual value CAN frame.
//!
//! Decodes the data stored in the frame, assuming that the frame contains
//! eight bytes of which the lower four represent one value and the higher four
//! another value.  Further it is assumed that the data in the frame is decoded
//! as Q24 value which is converted to float.
//!
//! \param frame The CAN frame.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param out   The decoded data is written to `out`.  The value of the lower
//!              bytes is stored in out->value[0], the value of the higher
//!              bytes in out->value[1].
void BLMC_decodeCanMotorMsg(frame_t const * const frame,
        nanosecs_abs_t timestamp, BLMC_StampedValue_t *out);


//! \brief Decode a status message from a CAN frame.
//! \param frame The CAN frame.
//! \param status Data from the frame is stored here.
void BLMC_decodeCanStatusMsg(frame_t const * const frame,
        BLMC_StatusMsg_t *status);


//! \brief Update board data with status frame.
//! \param frame CAN frame that contains status message.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param bd Board data structure.
void BLMC_updateStatus(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


//! \brief Update board data with motor current frame.
//! \param frame CAN frame that contains motor current values.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param bd Board data structure.
void BLMC_updateCurrent(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


//! \brief Update board data with motor position frame.
//! \param frame CAN frame that contains motor position values.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param bd Board data structure.
void BLMC_updatePosition(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


//! \brief Update board data with motor velocity frame.
//! \param frame CAN frame that contains motor velocity values.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param bd Board data structure.
void BLMC_updateVelocity(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


//! \brief Update board data with ADC6 result frame.
//! \param frame CAN frame that contains ADC6 values.
//! \param timestamp Timestamp of the moment the frame was received.
//! \param bd Board data structure.
void BLMC_updateAdc6(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


//! \brief Print the current board status in a human readable way.
void BLMC_printBoardStatus(BLMC_BoardData_t const * const bd);


//! \brief Send the frame currently stored in the handle
//!
//! This function assumes that the `frame` of handle is setup as desired. It is
//! meant as a internal helper functions and not for direct use. Use
//! BLMC_sendCommand() or BLMC_sendMotorCurrents() instead.
//!
//! \param handle The CAN connection handle.
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int BLMC_sendCurrentFrame(BLMC_CanHandle_t handle);


//! \brief Send a command to the board
//!
//! Send a command message with specified command id and value.  Please use the
//! BLMC_CMD_* defines for the id:
//!
//!     BLMC_sendCommand(canHandle, BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
//!
//! \param handle The CAN connection handle.
//! \param cmd_id Command ID. See the BLMC_CMD_* defines for possible values.
//! \param value  The value to be set.  For binary commands use BLMC_ENABLE or
//!               BLMC_DISABLE.
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int BLMC_sendCommand(BLMC_CanHandle_t handle, uint32_t cmd_id, int32_t value);


//! \brief Send a motor current command to the board
//! \param handle The CAN connection handle.
//! \param current_mtr1 Reference current for motor 1
//! \param current_mtr2 Reference current for motor 2
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int BLMC_sendMotorCurrent(BLMC_CanHandle_t handle, float current_mtr1,
        float current_mtr2);


//! \brief Receive message and update board data
//!
//! Receives one message from the CAN bus and updates the board data structure
//! accordingly.  If the received message is not related to the board data, it
//! is dropped without notice!
//!
//! \param handle The CAN connection handle.
//! \param board_data [in|out] The board data structure.
//! \returns The return value of the rt_dev_recv function (< 0 in case of
//!          error).
int BLMC_receiveBoardMessage(BLMC_CanHandle_t handle,
        BLMC_BoardData_t *board_data);

#endif // BLMC_CAN_H_
