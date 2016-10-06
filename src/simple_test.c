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
#define BLMC_CAN_ID_COMMANDS   0x00
#define BLMC_CAN_ID_IqRef      0x05
#define BLMC_CAN_ID_STATUSMSG  0x10
#define BLMC_CAN_ID_Iq         0x20
#define BLMC_CAN_ID_POS        0x30 // FIXME fix this on board code!
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

#define BLMC_MTR1 0
#define BLMC_MTR2 1

#define BLMC_ADC_A 0
#define BLMC_ADC_B 1

// TODO rename struct and some of the flags
//! \brief Status message bits.
typedef struct _BLMC_StatusMsg_t_
{                             // bits
   uint8_t system_enabled:1;  // 0
   uint8_t motor1_enabled:1;  // 1
   uint8_t motor1_ready:1;    // 2
   uint8_t motor2_enabled:1;  // 3
   uint8_t motor2_ready:1;    // 4
   uint8_t error_code:3;      // 5-7
} BLMC_StatusMsg_t;


typedef struct _BLMC_StampedValue_t_
{
    float value[2];
    uint32_t timestamp;
} BLMC_StampedValue_t;


typedef struct _BLMC_BoardData_t_
{
    BLMC_StatusMsg_t status;
    BLMC_StampedValue_t current;
    BLMC_StampedValue_t position;
    BLMC_StampedValue_t velocity;
    BLMC_StampedValue_t adc6;
} BLMC_BoardData_t;

typedef struct can_frame frame_t;


BLMC_BoardData_t board_data;

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


void BLMC_decodeCanMotorMsg(frame_t frame, BLMC_StampedValue_t *out)
{
    out->timestamp = 42;  // FIXME
    out->value[BLMC_MTR1] = QBYTES_TO_FLOAT(frame.data);
    out->value[BLMC_MTR2] = QBYTES_TO_FLOAT((frame.data + 4));
}

void BLMC_decodeCanStatusMsg(const struct can_frame frame,
        BLMC_StatusMsg_t *status)
{
    // We only have one byte of data
    uint8_t data = frame.data[0];

    status->system_enabled = data >> 0;
    status->motor1_enabled = data >> 1;
    status->motor1_ready   = data >> 2;
    status->motor2_enabled = data >> 3;
    status->motor2_ready   = data >> 4;
    status->error_code     = data >> 5;
}

void BLMC_updateStatus(frame_t frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanStatusMsg(frame, &bd->status);
}

void BLMC_updateCurrent(frame_t frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->current);
}

void BLMC_updatePosition(frame_t frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->position);
}

void BLMC_updateVelocity(frame_t frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->velocity);
}

void BLMC_updateAdc6(frame_t frame, BLMC_BoardData_t *bd)
{
    BLMC_decodeCanMotorMsg(frame, &bd->adc6);
}

void print_motor_position(struct can_frame frame)
{
    float pos_mtr1, pos_mtr2;

    // big endian:
    pos_mtr1 = QBYTES_TO_FLOAT(frame.data);
    pos_mtr2 = QBYTES_TO_FLOAT((frame.data + 4));

    printf("Position: mtr1 = %f, mtr2 = %f", pos_mtr1, pos_mtr2);
}



void print_status(struct can_frame frame)
{
    BLMC_StatusMsg_t status;
    BLMC_decodeCanStatusMsg(frame, &status);

    printf("Status:\n");
    printf("\tSystem enabled: %d\n", status.system_enabled);
    printf("\tMotor 1 enabled: %d\n", status.motor1_enabled);
    printf("\tMotor 1 ready: %d\n", status.motor1_ready);
    printf("\tMotor 2 enabled: %d\n", status.motor2_enabled);
    printf("\tMotor 2 ready: %d\n", status.motor2_ready);
    printf("\tError: %d\n", status.error_code);
}


void BLMC_printBoardStatus(const BLMC_BoardData_t* const bd)
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
        printf("Motor %d", i+1);
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
    socklen_t addrlen = sizeof(addr);

    while (1) {
        ret = rt_dev_recvfrom(s, (void *)&frame, sizeof(can_frame_t), 0,
                              (struct sockaddr *)&addr, &addrlen);
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
                // current
                BLMC_updateCurrent(frame, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_POS) {
                // motor positions
                BLMC_updatePosition(frame, &board_data);
                //print_motor_position(frame);
            } else if (frame.can_id == BLMC_CAN_ID_SPEED) {
                BLMC_updateVelocity(frame, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_ADC6) {
                BLMC_updateAdc6(frame, &board_data);
            } else if (frame.can_id == BLMC_CAN_ID_STATUSMSG) {
                BLMC_updateStatus(frame, &board_data);
                //print_status(frame);
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
