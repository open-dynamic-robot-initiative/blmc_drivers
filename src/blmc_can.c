#include "blmc_can.h"
#include <stdio.h>
#include <string.h>  // memset


inline BLMC_CanHandle_t BLMC_initCanHandle(BLMC_CanConnection_t *can_con)
{
    return (BLMC_CanHandle_t) can_con;
}

void BLMC_initCan(BLMC_CanHandle_t canHandle)
{
    BLMC_CanConnection_t *can = (BLMC_CanConnection_t*)canHandle;

    //can->recv_addr.can_family = AF_CAN;
    //can->recv_addr.can_ifindex = ?

    can->msg.msg_iov = &can->iov;
    can->msg.msg_iovlen = 1;
    can->msg.msg_name = (void *)&can->msg_addr;
    can->msg.msg_namelen = sizeof(struct sockaddr_can);
    can->msg.msg_control = (void *)&can->timestamp;
    can->msg.msg_controllen = sizeof(nanosecs_abs_t);

    // TODO why the memset?
    memset(&can->send_addr, 0, sizeof(can->send_addr));
    can->send_addr.can_family = AF_CAN;
    can->send_addr.can_ifindex = 1;  // TODO do not hard code!
}

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

void BLMC_decodeCanMotorMsg(frame_t const * const frame,
        nanosecs_abs_t timestamp, BLMC_StampedValue_t *out)
{
    // TODO rename function (no motor, more dual msg, float, Q24)
    out->timestamp = timestamp;  // FIXME
    out->value[BLMC_MTR1] = QBYTES_TO_FLOAT(frame->data);
    out->value[BLMC_MTR2] = QBYTES_TO_FLOAT((frame->data + 4));
}

void BLMC_decodeCanStatusMsg(frame_t const * const frame,
        BLMC_StatusMsg_t *status)
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

void BLMC_updateStatus(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd)
{
    // NOTE: timestamp of status messages is currently not stored.
    BLMC_decodeCanStatusMsg(frame, &bd->status);
}

void BLMC_updateCurrent(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, timestamp, &bd->current);
}

void BLMC_updatePosition(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd)
{
    //printf("time since last position: %.3f ms\n",
    //        (timestamp - bd->position.timestamp)/1e6);
    BLMC_decodeCanMotorMsg(frame, timestamp, &bd->position);
}

void BLMC_updateVelocity(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, timestamp, &bd->velocity);
}

void BLMC_updateAdc6(frame_t const * const frame, nanosecs_abs_t timestamp,
        BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, timestamp, &bd->adc6);
}


void BLMC_printBoardStatus(BLMC_BoardData_t const * const bd)
{
    int i;

    printf("System:\n");
    printf("\tSystem enabled: %d\n", bd->status.system_enabled);
    printf("\tMotor 1 enabled: %d\n", bd->status.motor1_enabled);
    printf("\tMotor 1 ready: %d\n", bd->status.motor1_ready);
    printf("\tMotor 2 enabled: %d\n", bd->status.motor2_enabled);
    printf("\tMotor 2 ready: %d\n", bd->status.motor2_ready);
    printf("\tError: %d\n", bd->status.error_code);

    for (i = 0; i < 2; ++i) {
        printf("Motor %d\n", i+1);
        if (bd->current.timestamp)
            printf("\tCurrent: %f\n", bd->current.value[i]);
        if (bd->position.timestamp)
            printf("\tPosition: %f\n", bd->position.value[i]);
        if (bd->velocity.timestamp)
            printf("\tVelocity: %f\n", bd->velocity.value[i]);
    }
}

int BLMC_sendCommand(BLMC_CanHandle_t handle, uint32_t cmd_id, int32_t value)
{
    int ret;
    BLMC_CanConnection_t *can = (BLMC_CanConnection_t*)handle;

    // Fill frame
    // ----------

    // header
    can->frame.can_id = BLMC_CAN_ID_COMMAND;
    can->frame.can_dlc = 8;  // number of bytes

    // value
    can->frame.data[0] = (value >> 24) & 0xFF;
    can->frame.data[1] = (value >> 16) & 0xFF;
    can->frame.data[2] = (value >> 8) & 0xFF;
    can->frame.data[3] = value & 0xFF;

    // command
    can->frame.data[4] = (cmd_id >> 24) & 0xFF;
    can->frame.data[5] = (cmd_id >> 16) & 0xFF;
    can->frame.data[6] = (cmd_id >> 8) & 0xFF;
    can->frame.data[7] = cmd_id & 0xFF;

    ret = rt_dev_sendto(can->socket, (void *)&can->frame, sizeof(can_frame_t),
            0, (struct sockaddr *)&can->send_addr, sizeof(can->send_addr));
    return ret;
}

int BLMC_receiveBoardMessage(BLMC_CanHandle_t handle,
        BLMC_BoardData_t *board_data)
{
    int ret;
    BLMC_CanConnection_t *can = (BLMC_CanConnection_t*)handle;


    can->iov.iov_base = (void *)&can->frame;
    can->iov.iov_len = sizeof(can_frame_t);

    ret = rt_dev_recvmsg(can->socket, &can->msg, 0);

    if (can->msg.msg_controllen == 0) {
        // No timestamp for this frame available. Make sure we dont get
        // garbage.
        can->timestamp = 0;
    }

    if (ret >= 0) {
        if (can->frame.can_id == BLMC_CAN_ID_Iq) {
            BLMC_updateCurrent(&can->frame, can->timestamp, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_POS) {
            BLMC_updatePosition(&can->frame, can->timestamp, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_SPEED) {
            BLMC_updateVelocity(&can->frame, can->timestamp, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_ADC6) {
            BLMC_updateAdc6(&can->frame, can->timestamp, board_data);
        } else if (can->frame.can_id == BLMC_CAN_ID_STATUSMSG) {
            BLMC_updateStatus(&can->frame, can->timestamp, board_data);
        }
    }

    return ret;
}
