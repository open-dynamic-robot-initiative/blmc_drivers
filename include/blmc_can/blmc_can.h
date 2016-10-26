/**
 * \file
 * \brief API for the CAN interface of the Motor Control Microcontroller Boards
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 *
 * \defgroup MotorAPI Motor API
 * \brief API for communication with the microcontroller board that drives the
 *        motors.
 *
 * Usage
 * -----
 *
 * ### Receive Data
 *
 * \code{C}
 *     CAN_CanConnection_t can_con;
 *     CAN_CanHandle_t can_handle;
 *     BLMC_BoardData_t board_data;
 *     CAN_Frame_t frame;
 *
 *     // initialize
 *     can_handle = CAN_initCanHandle(&can_con);
 *     BLMC_initBoardData(&board_data, BLMC_SYNC_ON_ADC6);
 *     CAN_setupCan(can_handle, "rtcan0", 0);
 *
 *     // enable all sensor messages
 *     BLMC_sendCommand(can_handle, BLMC_CMD_SEND_ALL, BLMC_ENABLE);
 *
 *     // receive & process messages
 *     while (1) {
 *       CAN_receiveFrame(can_handle, &frame);
 *       BLMC_processCanFrame(&frame, &board_data);
 *
 *       BLMC_printSynchronizedBoardStatus(&board_data);
 *     }
 * \endcode
 *
 *
 * ### Send Commands
 *
 * \code{C}
 *     CAN_CanConnection_t can_con;
 *     CAN_CanHandle_t can_handle;
 *     BLMC_BoardData_t board_data;
 *     CAN_Frame_t frame;
 *
 *     // initialize
 *     can_handle = CAN_initCanHandle(&can_con);
 *     BLMC_initBoardData(&board_data, BLMC_SYNC_ON_ADC6);
 *     CAN_setupCan(can_handle, "rtcan0", 0);
 *
 *     // enable system and motor 1
 *     BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);
 *     BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
 *
 *     // send current command to motor 1
 *     BLMC_sendMotorCurrent(can_handle, 0.5, 0);
 * \endcode
 *
 * See also the file `src/blmc_can_demo.c` for a more complex example.
 *
 *
 * Synchronization
 * ---------------
 *
 * \anchor Synchronization
 *
 * The different sensor messages are sampled at the same time on the board and
 * then sent in a squence in the following order: current, position, velocity,
 * adc6.
 *
 * To make sure that the values of one such squence are processed together, a
 * simple synchronization technique is implemented in this module.  The
 * BLMC_BoardData_t structure has two members `latest` and `sync` that both
 * contain the sensor data.
 *
 * In `latest` always the latest messages are stored (thus the name)
 * immediately when they arrive.  This means that `latest.current` may already
 * contain a new current measurement while `latest.position` still contains the
 * value from the previous time step.  Once the last message of a sequence is
 * received, the content of `latest` is copied to `sync`.  This means that
 * `sync` always contains values that belong to the same time step.
 *
 * ### Synchronization Trigger
 *
 * Since individual sensor messages can be disabled on the board, it is not
 * generally determined which message marks the end of the sequence and should
 * trigger the synchronization.  Therefore this message has to be specified
 * manually by setting the `sync_trigger` argument of BLMC_initBoardData() to
 * the correct value.  In the usual case where all messages are enabled, this
 * should be `BLMC_SYNC_ON_ADC6`.  Make sure to set this correctly, otherwise
 * the data in `sync` is not synchronized correctly.
 *
 * \{
 */
#ifndef BLMC_CAN_H_
#define BLMC_CAN_H_

// INCLUDES
// **************************************************************************
#include <blmc_can/can.h>


#ifdef __cplusplus
extern "C"{
#endif


// DEFINES
// **************************************************************************

//! \name Arbitration IDs
//! \brief Arbitration IDs of the different message types.
//! \{
#define BLMC_CAN_ID_COMMAND    0x00
#define BLMC_CAN_ID_IqRef      0x05
#define BLMC_CAN_ID_STATUSMSG  0x10
#define BLMC_CAN_ID_Iq         0x20
#define BLMC_CAN_ID_POS        0x30
#define BLMC_CAN_ID_SPEED      0x40
#define BLMC_CAN_ID_ADC6       0x50
//! \}


//! \name Command IDs
//! \anchor CommandIDs
//! \brief IDs of the various commands that can be sent with BLMC_sendCommand().
//! \{
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
#define BLMC_CMD_SET_CAN_RECV_TIMEOUT 30
//! \}

//! \name Command Values
//! \anchor CommandValues
//! \brief Possible values for commands that only take a boolean value.
//! \see BLMC_sendCommand(), \ref CommandIDs
//! \{
#define BLMC_ENABLE 1
#define BLMC_DISABLE 0
//! \}


//! \name Synchronization options
//! Use these to specify the message type on which sensor data is synchronized.
//!
//! \{

//! \brief No synchronization. If this is set, BoardData_t.sync is undefined.
#define BLMC_SYNC_DISABLED 0
//! \brief Synchronize after receiving a current message.
#define BLMC_SYNC_ON_CURRENT 1
//! \brief Synchronize after receiving a position message.
#define BLMC_SYNC_ON_POSITION 2
//! \brief Synchronize after receiving a velocity message.
#define BLMC_SYNC_ON_VELOCITY 3
//! \brief Synchronize after receiving a ADC6 message.
#define BLMC_SYNC_ON_ADC6 4
//! \}


//! \name Motor Indices
//! \{
#define BLMC_MTR1 0
#define BLMC_MTR2 1
//! \}

//! \name ADC Indices
//! \{
#define BLMC_ADC_A 0
#define BLMC_ADC_B 1
//! \}


//! \name Board Error Codes
//! \brief Possible error codes in the status message
//! \{

//! \brief No error
#define BLMC_BOARD_ERROR_NONE 0
//! \brief Encoder error too high
#define BLMC_BOARD_ERROR_ENCODER 1
//! \brief Timeout for receiving current references exceeded
#define BLMC_BOARD_ERROR_CAN_RECV_TIMEOUT 2
//! \brief Motor temperature reached critical value
//! \note This is currently unused as no temperature sensing is done.
#define BLMC_BOARD_ERROR_CRIT_TEMP 3  // currently unused
//! \brief Some other error
#define BLMC_BOARD_ERROR_OTHER 7
//! \}


// TYPEDEFS
// **************************************************************************

typedef CAN_Frame_t const * const const_frame_ptr;

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


//! \brief Bundles various sensor data from the board
typedef struct _BLMC_SensorData_t_
{
    BLMC_StampedValue_t current;
    BLMC_StampedValue_t position;
    BLMC_StampedValue_t velocity;
    BLMC_StampedValue_t adc6;
} BLMC_SensorData_t;


//! \brief Bundles all data send by the board.
typedef struct _BLMC_BoardData_t_
{
    //! Last received status message.
    BLMC_StatusMsg_t status;
    //! Last received sensor messages.
    BLMC_SensorData_t latest;
    //! Synchronized sensor messages.
    BLMC_SensorData_t sync;
    //! Specifies the message type on which messages are synchronized.
    uint8_t sync_trigger;
} BLMC_BoardData_t;


// FUNCTIONS
// **************************************************************************


//! \brief Initialize a stamped value to zero.
void BLMC_initStampedValue(BLMC_StampedValue_t *sv);


//! \brief Initialize a sensor data struct.
void BLMC_initSensorData(BLMC_SensorData_t *sd);


//! \brief Initialize status message (set everything to zero).
void BLMC_initStatus(BLMC_StatusMsg_t *st);


//! \brief Initialize board data structure.
//!
//! \see \ref Synchronization
//! \param bd Board data instance
//! \param sync_trigger Specified the message type on which messages are
//                      sychnronized. Use the BLMC_SYNC_* constants.
void BLMC_initBoardData(BLMC_BoardData_t *bd, uint8_t sync_trigger);


//! Set a new synchronization trigger.
void BLMC_setSyncTrigger(BLMC_BoardData_t *bd, uint8_t trigger);


//! \brief Copy latest sensor data to synchronized.  Do not use this manually.
void BLMC_synchronize(BLMC_BoardData_t *bd);


//! \brief Decode a dual value CAN frame.
//!
//! Decodes the data stored in the frame, assuming that the frame contains
//! eight bytes of which the lower four represent one value and the higher four
//! another value.  Further it is assumed that the data in the frame is decoded
//! as Q24 value which is converted to float.
//!
//! \param frame The CAN frame.
//! \param out   The decoded data is written to `out`.  The value of the lower
//!              bytes is stored in out->value[0], the value of the higher
//!              bytes in out->value[1].
void BLMC_decodeCanMotorMsg(const_frame_ptr frame, BLMC_StampedValue_t *out);


//! \brief Decode a status message from a CAN frame.
//! \param frame The CAN frame.
//! \param status Data from the frame is stored here.
void BLMC_decodeCanStatusMsg(const_frame_ptr frame, BLMC_StatusMsg_t *status);


//! \brief Update board data with status frame.
//! \param frame CAN frame that contains status message.
//! \param bd Board data structure.
void BLMC_updateStatus(const_frame_ptr frame, BLMC_BoardData_t *bd);


//! \brief Update board data with motor current frame.
//! \param frame CAN frame that contains motor current values.
//! \param bd Board data structure.
void BLMC_updateCurrent(const_frame_ptr frame, BLMC_BoardData_t *bd);


//! \brief Update board data with motor position frame.
//! \param frame CAN frame that contains motor position values.
//! \param bd Board data structure.
void BLMC_updatePosition(const_frame_ptr frame, BLMC_BoardData_t *bd);


//! \brief Update board data with motor velocity frame.
//! \param frame CAN frame that contains motor velocity values.
//! \param bd Board data structure.
void BLMC_updateVelocity(const_frame_ptr frame, BLMC_BoardData_t *bd);


//! \brief Update board data with ADC6 result frame.
//! \param frame CAN frame that contains ADC6 values.
//! \param bd Board data structure.
void BLMC_updateAdc6(const_frame_ptr frame, BLMC_BoardData_t *bd);


//! \brief Print the latest board status in a human readable way.
void BLMC_printLatestBoardStatus(BLMC_BoardData_t const * const bd);


//! \brief Print the synchronized board status in a human readable way.
void BLMC_printSynchronizedBoardStatus(BLMC_BoardData_t const * const bd);


void BLMC_printStatus(BLMC_StatusMsg_t const *status);


void BLMC_printSensorData(BLMC_SensorData_t const *data);


//! \brief Send a command to the board
//!
//! Send a command message with specified command id and value.  Use the
//! BLMC_CMD_* defines for the ID:
//!
//!     BLMC_sendCommand(canHandle, BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
//!
//! \see \ref CommandIDs, \ref CommandValues
//! \param handle The CAN connection handle.
//! \param cmd_id Command ID. See the BLMC_CMD_* defines for possible values.
//! \param value  The value to be set.  For binary commands use BLMC_ENABLE or
//!               BLMC_DISABLE.
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int BLMC_sendCommand(CAN_CanHandle_t handle, uint32_t cmd_id, int32_t value);


//! \brief Send a motor current command to the board
//! \param handle The CAN connection handle.
//! \param current_mtr1 Reference current for motor 1
//! \param current_mtr2 Reference current for motor 2
//! \returns The return value of the rt_dev_send function (< 0 in case of
//!          error).
int BLMC_sendMotorCurrent(CAN_CanHandle_t handle, float current_mtr1,
        float current_mtr2);


//! \brief Process a CAN frame
//!
//! Checks if the frame contains data related to the board and, if yes, updates
//! board_data.
//!
//! \param frame The CAN frame that is to be processed.
//! \param[out] board_data If the frame contains data related to the board,
//!                        board_data is updated accordingly
//! \returns 0 if the frame contained board data, 1 if not.
int BLMC_processCanFrame(const_frame_ptr frame, BLMC_BoardData_t *board_data);


//! \brief Get a meaningful error name for a given error code
//! \param[in]  error_code The error code sent by the board.
//! \param[out] error_name Human-readable error name.
void BLMC_getErrorName(uint8_t error_code, char* error_name);

#ifdef __cplusplus
}
#endif

/** \} */ // end group MotorAPI

#endif // BLMC_CAN_H_
