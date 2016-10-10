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

// Motor Indices
#define BLMC_MTR1 0
#define BLMC_MTR2 1

// ADC Indices
#define BLMC_ADC_A 0
#define BLMC_ADC_B 1



// Convertion of a byte array to a int32_t.
#define BYTES_TO_INT32(bytes) (\
        (int32_t) bytes[3] + \
        ((int32_t)bytes[2] << 8) + \
        ((int32_t)bytes[1] << 16) + \
        ((int32_t)bytes[0] << 24) \
        )

// Convertion of Q24 value to float.
#define Q24_TO_FLOAT(qval) ((float)qval / (1 << 24))

// Convertion of Q24 byte array to float.
#define QBYTES_TO_FLOAT(qbytes) (\
        Q24_TO_FLOAT( BYTES_TO_INT32(qbytes) ) )


// TYPEDEFS
// **************************************************************************


//! \brief Status Message
typedef struct _BLMC_StatusMsg_t_
{                             // bits
   uint8_t system_enabled:1;  // 0
   uint8_t motor1_enabled:1;  // 1
   uint8_t motor1_ready:1;    // 2
   uint8_t motor2_enabled:1;  // 3
   uint8_t motor2_ready:1;    // 4
   uint8_t error_code:3;      // 5-7
} BLMC_StatusMsg_t;


// TODO better name (float and [2])
typedef struct _BLMC_StampedValue_t_
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

// just for convenience
typedef struct can_frame frame_t;




// FUNCTIONS
// **************************************************************************


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


//! Update board data with status frame.
void BLMC_updateStatus(frame_t const * const frame, BLMC_BoardData_t *bd);


void BLMC_updateCurrent(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


void BLMC_updatePosition(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


void BLMC_updateVelocity(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


void BLMC_updateAdc6(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd);


void BLMC_printBoardStatus(BLMC_BoardData_t const * const bd);


#endif // BLMC_CAN_H_
