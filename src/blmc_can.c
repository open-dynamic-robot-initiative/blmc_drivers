#include <blmc_can/blmc_can.h>
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

void BLMC_initSensorData(BLMC_SensorData_t *sd)
{
    BLMC_initStampedValue(&sd->current);
    BLMC_initStampedValue(&sd->position);
    BLMC_initStampedValue(&sd->velocity);
    BLMC_initStampedValue(&sd->adc6);
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

void BLMC_initStampedStatus(BLMC_StampedStatus_t *st)
{
    st->timestamp = 0;
    BLMC_initStatus(&st->status);
}

void BLMC_initBoardData(BLMC_BoardData_t *bd, uint8_t sync_trigger)
{
    BLMC_initStampedStatus(&bd->status);
    BLMC_initSensorData(&bd->latest);
    BLMC_initSensorData(&bd->sync);
    bd->sync_trigger = sync_trigger;
}

void BLMC_setSyncTrigger(BLMC_BoardData_t *bd, uint8_t trigger)
{
    bd->sync_trigger = trigger;
}

void BLMC_synchronize(BLMC_BoardData_t *bd)
{
    bd->sync = bd->latest;
}

void BLMC_decodeCanMotorMsg(const_frame_ptr frame, BLMC_StampedValue_t *out)
{
    // TODO rename function (no motor, more dual msg, float, Q24)
    out->timestamp = frame->timestamp;
    out->value[BLMC_MTR1] = QBYTES_TO_FLOAT(frame->data);
    out->value[BLMC_MTR2] = QBYTES_TO_FLOAT((frame->data + 4));
}

void BLMC_decodeCanStatusMsg(const_frame_ptr frame,
        BLMC_StampedStatus_t *status)
{
    // We only have one byte of data
    uint8_t data = frame->data[0];

    status->timestamp = frame->timestamp;
    status->status.system_enabled = data >> 0;
    status->status.motor1_enabled = data >> 1;
    status->status.motor1_ready   = data >> 2;
    status->status.motor2_enabled = data >> 3;
    status->status.motor2_ready   = data >> 4;
    status->status.error_code     = data >> 5;
}

void BLMC_updateStatus(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    // NOTE: timestamp of status messages is currently not stored.
    BLMC_decodeCanStatusMsg(frame, &bd->status);
}

void BLMC_updateCurrent(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->latest.current);
    if (bd->sync_trigger == BLMC_SYNC_ON_CURRENT) {
        BLMC_synchronize(bd);
    }
}

void BLMC_updatePosition(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->latest.position);
    if (bd->sync_trigger == BLMC_SYNC_ON_POSITION) {
        BLMC_synchronize(bd);
    }
}

void BLMC_updateVelocity(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->latest.velocity);
    if (bd->sync_trigger == BLMC_SYNC_ON_VELOCITY) {
        BLMC_synchronize(bd);
    }
}

void BLMC_updateAdc6(const_frame_ptr frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->latest.adc6);
    if (bd->sync_trigger == BLMC_SYNC_ON_ADC6) {
        BLMC_synchronize(bd);
    }
}

void BLMC_printLatestBoardStatus(BLMC_BoardData_t const * const bd)
{
    BLMC_printStatus(&bd->status.status);
    BLMC_printSensorData(&bd->latest);
}

void BLMC_printSynchronizedBoardStatus(BLMC_BoardData_t const * const bd)
{
    BLMC_printStatus(&bd->status.status);
    BLMC_printSensorData(&bd->sync);
}

void BLMC_printStatus(BLMC_StatusMsg_t const *status)
{
    rt_printf("System:\n");
    rt_printf("\tSystem enabled: %d\n", status->system_enabled);
    rt_printf("\tMotor 1 enabled: %d\n", status->motor1_enabled);
    rt_printf("\tMotor 1 ready: %d\n", status->motor1_ready);
    rt_printf("\tMotor 2 enabled: %d\n", status->motor2_enabled);
    rt_printf("\tMotor 2 ready: %d\n", status->motor2_ready);
    rt_printf("\tError: %d\n", status->error_code);
}


void BLMC_printSensorData(BLMC_SensorData_t const *data)
{
    int i;

    for (i = 0; i < 2; ++i) {
        rt_printf("Motor %d\n", i+1);
        if (data->current.timestamp)
            rt_printf("\tCurrent: %f\n", data->current.value[i]);
        if (data->position.timestamp)
            rt_printf("\tPosition: %f\n", data->position.value[i]);
        if (data->velocity.timestamp)
            rt_printf("\tVelocity: %f\n", data->velocity.value[i]);
    }

    rt_printf("ADC\n\tA6: %f\n\tB6: %f\n", data->adc6.value[0],
            data->adc6.value[1]);
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

int BLMC_processCanFrame(const_frame_ptr frame,
        BLMC_BoardData_t *board_data)
{

    if (frame->id == BLMC_CAN_ID_Iq) {
        BLMC_updateCurrent(frame, board_data);
    } else if (frame->id == BLMC_CAN_ID_POS) {
        BLMC_updatePosition(frame, board_data);
    } else if (frame->id == BLMC_CAN_ID_SPEED) {
        BLMC_updateVelocity(frame, board_data);
    } else if (frame->id == BLMC_CAN_ID_ADC6) {
        BLMC_updateAdc6(frame, board_data);
    } else if (frame->id == BLMC_CAN_ID_STATUSMSG) {
        BLMC_updateStatus(frame, board_data);
    } else {
        // no frame for me
        return 1;  // FIXME no magic numbers
    }

    // it was a frame for me
    return 0;
}

void BLMC_getErrorName(uint8_t error_code, char* error_name)
{
    // NOTE: Error names must not exceed 30 chars
    switch (error_code) {
        case BLMC_BOARD_ERROR_NONE:
            strcpy(error_name, "No Error");
            break;
        case BLMC_BOARD_ERROR_ENCODER:
            strcpy(error_name, "Encoder Error");
            break;
        case BLMC_BOARD_ERROR_CAN_RECV_TIMEOUT:
            strcpy(error_name, "CAN Receive Timeout");
            break;
        case BLMC_BOARD_ERROR_CRIT_TEMP:
            strcpy(error_name, "Critical Motor Temperature");
            break;
        case BLMC_BOARD_ERROR_OTHER:
            strcpy(error_name, "Other Error");
            break;
        default:
            strcpy(error_name, "Unknown Error");
    }
}
