/*
 * Program to receive CAN messages
 *
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
//#include <time.h>
//#include <errno.h>
//#include <getopt.h>
#include <sys/mman.h>
#include <native/task.h>
//#include <native/pipe.h>
#include <rtdm/rtcan.h>


extern int optind, opterr, optopt;
static int s = -1, verbose = 0, print = 1;
RT_TASK rt_task_desc;
#define BUF_SIZ 255
#define MAX_FILTER 16
struct sockaddr_can recv_addr;
struct can_filter recv_filter[MAX_FILTER];
static int filter_count = 0;

int add_filter(u_int32_t id, u_int32_t mask)
{
    if (filter_count >= MAX_FILTER)
        return -1;
    recv_filter[filter_count].can_id = id;
    recv_filter[filter_count].can_mask = mask;
    printf("Filter #%d: id=0x%08x mask=0x%08x\n", filter_count, id, mask);
    filter_count++;
    return 0;
}

void cleanup(void)
{
    int ret;
    if (verbose)
        printf("Cleaning up...\n");
    if (s >= 0) {
        ret = rt_dev_close(s);
        s = -1;
        if (ret) {
            fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
        }
        exit(EXIT_SUCCESS);
    }
}

void cleanup_and_exit(int sig)
{
    if (verbose)
        printf("Signal %d received\n", sig);
    cleanup();
    exit(0);
}

#define BYTES_TO_INT32(bytes) (\
        (int32_t) bytes[3] + \
        ((int32_t)bytes[2] << 8) + \
        ((int32_t)bytes[1] << 16) + \
        ((int32_t)bytes[0] << 24) \
        )

#define Q24_TO_FLOAT(qval) ((float)qval / (1 << 24))

#define QBYTES_TO_FLOAT(qbytes) (\
        Q24_TO_FLOAT( BYTES_TO_INT32(qbytes) ) )

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

//! global board data
BLMC_BoardData_t board_data;

//! \brief Initialize a stamped value to zero.
void BLMC_initStampedValue(BLMC_StampedValue_t *sv)
{
    sv->timestamp = 0;
    sv->value[0] = 0.0;
    sv->value[1] = 0.0;
}

//! \brief Initialize status message (set everything to zero).
void BLMC_initStatus(BLMC_StatusMsg_t *st)
{
    st->system_enabled = 0;
    st->motor1_enabled = 0;
    st->motor1_ready   = 0;
    st->motor2_enabled = 0;
    st->motor2_ready   = 0;
    st->error_code     = 0;
}

//! \brief Initialize board data structure.
void BLMC_initBoardData(BLMC_BoardData_t *bd)
{
    BLMC_initStatus(&bd->status);
    BLMC_initStampedValue(&bd->current);
    BLMC_initStampedValue(&bd->position);
    BLMC_initStampedValue(&bd->velocity);
    BLMC_initStampedValue(&bd->adc6);
}

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
        nanosecs_abs_t timestamp, BLMC_StampedValue_t *out)
{
    // TODO rename function (no motor, more dual msg, float, Q24)
    out->timestamp = timestamp;  // FIXME
    out->value[BLMC_MTR1] = QBYTES_TO_FLOAT(frame->data);
    out->value[BLMC_MTR2] = QBYTES_TO_FLOAT((frame->data + 4));
}

//! \brief Decode a status message from a CAN frame.
//! \param frame The CAN frame.
//! \param status Data from the frame is stored here.
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

//! Update board data with status frame.
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


void rt_task(void)
{
    int i, ret, count = 0;
    struct can_frame frame;
    struct sockaddr_can addr;
    //socklen_t addrlen = sizeof(addr);
    struct msghdr msg;
    struct iovec iov;
    nanosecs_abs_t timestamp;

    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_name = (void *)&addr;
    msg.msg_namelen = sizeof(struct sockaddr_can);
    msg.msg_control = (void *)&timestamp;
    msg.msg_controllen = sizeof(nanosecs_abs_t);


    // Send a message to activate position messages
    {
        // define socket to which frame is sent
        struct sockaddr_can to_addr;
        memset(&to_addr, 0, sizeof(to_addr));
        to_addr.can_family = AF_CAN;
        to_addr.can_ifindex = 1;

        // fill frame with data
        frame.can_id = BLMC_CAN_ID_COMMAND;
        frame.can_dlc = 8;
        frame.data[0] = 0;
        frame.data[1] = 0;
        frame.data[2] = 0;
        frame.data[3] = 1;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 13;

        ret = rt_dev_sendto(s, (void *)&frame, sizeof(can_frame_t), 0,
                (struct sockaddr *)&to_addr, sizeof(to_addr));
        if (ret < 0) {
            switch (ret) {
                case -ETIMEDOUT:
                    if (verbose)
                        printf("rt_dev_send(to): timed out");
                    break;
                case -EBADF:
                    if (verbose)
                        printf("rt_dev_send(to): aborted because socket was closed");
                    break;
                default:
                    fprintf(stderr, "rt_dev_send: %s\n", strerror(-ret));
                    break;
            }
        }
    }

    while (1) {
        // without timestamps
        //ret = rt_dev_recvfrom(s, (void *)&frame, sizeof(can_frame_t), 0,
        //                      (struct sockaddr *)&addr, &addrlen);

        iov.iov_base = (void *)&frame;
        iov.iov_len = sizeof(can_frame_t);
        ret = rt_dev_recvmsg(s, &msg, 0);

        if (msg.msg_controllen == 0) {
            // No timestamp for this frame available. Make sure we dont get
            // garbage.
            timestamp = 0;
        }

        if (ret < 0) {
            switch (ret) {
            case -ETIMEDOUT:
                if (verbose)
                    printf("rt_dev_recv: timed out");
                continue;
            case -EBADF:
                if (verbose)
                    printf("rt_dev_recv: aborted because socket was closed");
                break;
            default:
                fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
            }
            break;
        }
        if (print && (count % print) == 0) {
            printf("#%d: (%d) ", count, addr.can_ifindex);

            if (frame.can_id == BLMC_CAN_ID_Iq) {
                BLMC_updateCurrent(&frame, timestamp, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_POS) {
                BLMC_updatePosition(&frame, timestamp, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_SPEED) {
                BLMC_updateVelocity(&frame, timestamp, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_ADC6) {
                BLMC_updateAdc6(&frame, timestamp, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_STATUSMSG) {
                BLMC_updateStatus(&frame, &board_data);
            } else {
                if (frame.can_id & CAN_ERR_FLAG)
                    printf("!0x%08x!", frame.can_id & CAN_ERR_MASK);
                else if (frame.can_id & CAN_EFF_FLAG)
                    printf("<0x%08x>", frame.can_id & CAN_EFF_MASK);
                else
                    printf("<0x%03x>", frame.can_id & CAN_SFF_MASK);
                printf(" [%d]", frame.can_dlc);
                if (!(frame.can_id & CAN_RTR_FLAG))
                    for (i = 0; i < frame.can_dlc; i++) {
                        printf(" %02x", frame.data[i]);
                    }
                if (frame.can_id & CAN_ERR_FLAG) {
                    printf(" ERROR ");
                    if (frame.can_id & CAN_ERR_BUSOFF)
                        printf("bus-off");
                    if (frame.can_id & CAN_ERR_CRTL)
                        printf("controller problem");
                } else if (frame.can_id & CAN_RTR_FLAG)
                    printf(" remote request");
            }
            BLMC_printBoardStatus(&board_data);
            printf("\n");
        }
        count++;
    }
}
int main(int argc, char **argv)
{
    int ret;
    //u_int32_t id, mask;
    u_int32_t err_mask = 0;
    struct ifreq ifr;
    //char *ptr;
    char name[32];

    mlockall(MCL_CURRENT | MCL_FUTURE);
    signal(SIGTERM, cleanup_and_exit);
    signal(SIGINT, cleanup_and_exit);

    //case 'f':
    /*
    ptr = optarg;
    while (1) {
        id = strtoul(ptr, NULL, 0);
        ptr = strchr(ptr, ':');
        if (!ptr) {
            fprintf(stderr, "filter must be applied in the form id:mask[:id:mask]...\n");
            exit(1);
        }
        ptr++;
        mask = strtoul(ptr, NULL, 0);
        ptr = strchr(ptr, ':');
        add_filter(id, mask);
        if (!ptr)
            break;
        ptr++;
    }
    */

    // timeout = (nanosecs_rel_t)strtoul(optarg, NULL, 0) * 1000000;

    BLMC_initBoardData(&board_data);

    ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (ret < 0) {
        fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
        return -1;
    }

    s = ret;
    if (argv[optind] == NULL) {
        if (verbose)
            printf("interface all\n");
        ifr.ifr_ifindex = 0;
    } else {
        if (verbose)
            printf("interface %s\n", argv[optind]);
        strncpy(ifr.ifr_name, argv[optind], IFNAMSIZ);
        if (verbose)
            printf("s=%d, ifr_name=%s\n", s, ifr.ifr_name);
        ret = rt_dev_ioctl(s, SIOCGIFINDEX, &ifr);
        if (ret < 0) {
            fprintf(stderr, "rt_dev_ioctl GET_IFINDEX: %s\n", strerror(-ret));
            goto failure;
        }
    }

    if (err_mask) {
        ret = rt_dev_setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                &err_mask, sizeof(err_mask));
        if (ret < 0) {
            fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
            goto failure;
        }
        if (verbose)
            printf("Using err_mask=%#x\n", err_mask);
    }

    if (filter_count) {
        ret = rt_dev_setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER,
                                &recv_filter, filter_count *
                                sizeof(struct can_filter));
        if (ret < 0) {
            fprintf(stderr, "rt_dev_setsockopt: %s\n", strerror(-ret));
            goto failure;
        }
    }

    recv_addr.can_family = AF_CAN;
    recv_addr.can_ifindex = ifr.ifr_ifindex;
    ret = rt_dev_bind(s, (struct sockaddr *)&recv_addr,
                      sizeof(struct sockaddr_can));
    if (ret < 0) {
        fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
        goto failure;
    }

    // Enable timestamps for frames
    ret = rt_dev_ioctl(s, RTCAN_RTIOC_TAKE_TIMESTAMP, RTCAN_TAKE_TIMESTAMPS);
    if (ret) {
        fprintf(stderr, "rt_dev_ioctl TAKE_TIMESTAMP: %s\n", strerror(-ret));
        goto failure;
    }

    snprintf(name, sizeof(name), "rtcanrecv-%d", getpid());
    ret = rt_task_shadow(&rt_task_desc, name, 0, 0);
    if (ret) {
        fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
        goto failure;
    }
    rt_task();
    /* never returns */
 failure:
    cleanup();
    return -1;
}
