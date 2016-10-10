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
#include <sys/mman.h>
#include <native/task.h>
#include <rtdm/rtcan.h>
#include "blmc_can.h"


// GLOBALS
// **************************************************************************

extern int optind, opterr, optopt;
static int verbose = 0, print = 1;
RT_TASK rt_task_desc;
#define BUF_SIZ 255
#define MAX_FILTER 16
struct sockaddr_can recv_addr;
struct can_filter recv_filter[MAX_FILTER];
static int filter_count = 0;

//! global board data
BLMC_BoardData_t board_data;

BLMC_CanConnection_t can_con;
BLMC_CanHandle_t can_handle;


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


void cleanup_and_exit(int sig)
{
    if (verbose)
        printf("Signal %d received\n", sig);
    // Disable system before closing connection
    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
    BLMC_closeCan(can_handle);
    exit(0);
}


void rt_task(void)
{
    int ret, count = 0;

    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);

    // Send a message to activate position messages
    ret = BLMC_sendCommand(can_handle, BLMC_CMD_SEND_CURRENT, BLMC_ENABLE);
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

    //BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_MTR2, BLMC_ENABLE);

    // Receive messages and print board status
    while (1) {
        //BLMC_sendMotorCurrent(can_handle, 0, 0.3);
        ret = BLMC_receiveBoardMessage(can_handle, &board_data);
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
            printf("#%d: (%d)\n", count, can_con.msg_addr.can_ifindex);
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


    // Initialize stuff
    // ----------------
    //
    can_handle = BLMC_initCanHandle(&can_con);
    BLMC_initBoardData(&board_data);

    ret = BLMC_setupCan(can_handle, NULL, err_mask);
    if (ret < 0) {
        printf("Could'nt setup CAN connection. Exit.");
        return -1;
    }


    snprintf(name, sizeof(name), "rtcanrecv-%d", getpid());
    ret = rt_task_shadow(&rt_task_desc, name, 0, 0);
    if (ret) {
        fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
        BLMC_closeCan(can_handle);
        return -1;
    }
    rt_task(); /* never returns */

    return 0;
}

