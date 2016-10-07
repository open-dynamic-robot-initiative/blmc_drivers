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
#include "blmc_can.h"


// GLOBALS
// **************************************************************************

extern int optind, opterr, optopt;
static int s = -1, verbose = 0, print = 1;
RT_TASK rt_task_desc;
#define BUF_SIZ 255
#define MAX_FILTER 16
struct sockaddr_can recv_addr;
struct can_filter recv_filter[MAX_FILTER];
static int filter_count = 0;

//! global board data
BLMC_BoardData_t board_data;


// FUNCTIONS
// **************************************************************************

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

