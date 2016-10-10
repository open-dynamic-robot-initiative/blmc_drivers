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

void BLMC_updateStatus(frame_t const * const frame, BLMC_BoardData_t *bd)
{
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

