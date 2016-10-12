#include "blmc_can.h"
#include <string.h>  // memset
#include <rtdk.h>  // rt_printf


// DEFINES
// **************************************************************************

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


// FUNCTIONS
// **************************************************************************

void BLMC_initStampedValue(BLMC_StampedValue_t *sv)
{
    sv->timestamp = 0;
    sv->value[0] = 0.0;
    sv->value[1] = 0.0;
}

void BLMC_initStatus(BLMC_StatusMsg_t *st)
{
    st->system_enabled = 0;
    st->motor1_enabled = 0;
    st->motor1_ready   = 0;
    st->motor2_enabled = 0;
    st->motor2_ready   = 0;
    st->error_code     = 0;
}

void BLMC_initBoardData(BLMC_BoardData_t *bd)
{
    BLMC_initStatus(&bd->status);
    BLMC_initStampedValue(&bd->current);
    BLMC_initStampedValue(&bd->position);
    BLMC_initStampedValue(&bd->velocity);
    BLMC_initStampedValue(&bd->adc6);
}

void BLMC_decodeCanMotorMsg(const_frame_ptr frame, BLMC_StampedValue_t *out)
{
    // TODO rename function (no motor, more dual msg, float, Q24)
    out->timestamp = frame->timestamp;
    out->value[BLMC_MTR1] = QBYTES_TO_FLOAT(frame->data);
    out->value[BLMC_MTR2] = QBYTES_TO_FLOAT((frame->data + 4));
}

void BLMC_decodeCanStatusMsg(const_frame_ptr frame, BLMC_StatusMsg_t *status)
{
    // We only have one byte of data
    uint8_t data = frame->data[0];

    status->system_enabled = data >> 0;
    status->motor1_enabled = data >> 1;
    status->motor1_ready   = data >> 2;
    status->motor2_enabled = data >> 3;
    status->motor2_ready   = data >> 4;
    status->error_code     = data >> 5;
}

void BLMC_updateStatus(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    // NOTE: timestamp of status messages is currently not stored.
    BLMC_decodeCanStatusMsg(frame, &bd->status);
}

void BLMC_updateCurrent(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->current);
}

void BLMC_updatePosition(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->position);
}

void BLMC_updateVelocity(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->velocity);
}

void BLMC_updateAdc6(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->adc6);
}


void BLMC_printBoardStatus(BLMC_BoardData_t const * const bd)
{
    int i;

    rt_printf("System:\n");
    rt_printf("\tSystem enabled: %d\n", bd->status.system_enabled);
    rt_printf("\tMotor 1 enabled: %d\n", bd->status.motor1_enabled);
    rt_printf("\tMotor 1 ready: %d\n", bd->status.motor1_ready);
    rt_printf("\tMotor 2 enabled: %d\n", bd->status.motor2_enabled);
    rt_printf("\tMotor 2 ready: %d\n", bd->status.motor2_ready);
    rt_printf("\tError: %d\n", bd->status.error_code);

    for (i = 0; i < 2; ++i) {
        rt_printf("Motor %d\n", i+1);
        if (bd->current.timestamp)
            rt_printf("\tCurrent: %f\n", bd->current.value[i]);
        if (bd->position.timestamp)
            rt_printf("\tPosition: %f\n", bd->position.value[i]);
        if (bd->velocity.timestamp)
            rt_printf("\tVelocity: %f\n", bd->velocity.value[i]);
    }
}

int BLMC_sendCommand(CAN_CanHandle_t handle, uint32_t cmd_id, int32_t value)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    uint8_t data[8];

    // value
    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;

    // command
    data[4] = (cmd_id >> 24) & 0xFF;
    data[5] = (cmd_id >> 16) & 0xFF;
    data[6] = (cmd_id >> 8) & 0xFF;
    data[7] = cmd_id & 0xFF;

    return CAN_sendFrame(handle, BLMC_CAN_ID_COMMAND, data, 8);
}

int BLMC_sendMotorCurrent(CAN_CanHandle_t handle, float current_mtr1,
        float current_mtr2)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    uint8_t data[8];
    uint32_t q_current1, q_current2;

    // Convert floats to Q24 values
    q_current1 = FLOAT_TO_Q24(current_mtr1);
    q_current2 = FLOAT_TO_Q24(current_mtr2);

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

    return CAN_sendFrame(handle, BLMC_CAN_ID_IqRef, data, 8);
}

int BLMC_receiveBoardMessage(CAN_CanHandle_t handle,
        BLMC_BoardData_t *board_data)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    int ret;
    CAN_Frame_t frame;

    ret = CAN_receiveFrame(handle, &frame);

    if (ret >= 0) {
        if (can->frame.can_id == BLMC_CAN_ID_Iq) {
            BLMC_updateCurrent(&frame, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_POS) {
            BLMC_updatePosition(&frame, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_SPEED) {
            BLMC_updateVelocity(&frame, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_ADC6) {
            BLMC_updateAdc6(&frame, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_STATUSMSG) {
            BLMC_updateStatus(&frame, board_data);
        }
    }

    return ret;
}
