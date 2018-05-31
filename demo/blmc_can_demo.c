/**
 * \brief Demo for the blmc_can library.
 *
 * This file shows how the blmc_can library can be used.  At start-up it sends
 * a few commands to the board to enable it and to make it send motor
 * information.  Then messages from the board (and optionally the OptoForce
 * sensor) are received and printed in a loop.
 *
 * While it has been changed in wide parts, this demo is originally based on
 * the rtcanrecv.c example file of xenomai (see copyright and licence below).
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
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#ifdef __XENO__
#include <native/task.h>
#include <rtdk.h>
#elif defined __RT_PREEMPT__
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>

/**
 * Computes the time as target = a - b;
 */
void timespec_sub(struct timespec *target, struct timespec *a, struct timespec *b)
{
    if (b->tv_nsec > a->tv_nsec) {
        target->tv_nsec = (a->tv_nsec - b->tv_nsec) + 1000000000;
        target->tv_sec = a->tv_sec - b->tv_sec - 1;
    } else {
        target->tv_nsec = a->tv_nsec - b->tv_nsec;
        target->tv_sec = a->tv_sec - b->tv_sec;
    }
}

#endif
#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>
#include <blmc_can/optoforce_can.h>


// GLOBALS
// **************************************************************************
const static int verbose = 0;

//#define MAX_FILTER 16
//struct can_filter recv_filter[MAX_FILTER];
//static int filter_count = 0;

CAN_CanConnection_t can_con;
CAN_CanHandle_t can_handle;

//! global board data
BLMC_BoardData_t board_data;

OPTO_OptoForceData_t opto_data;


// FUNCTIONS
// **************************************************************************


//int add_filter(u_int32_t id, u_int32_t mask)
//{
//    if (filter_count >= MAX_FILTER)
//        return -1;
//    recv_filter[filter_count].can_id = id;
//    recv_filter[filter_count].can_mask = mask;
//    rt_printf("Filter #%d: id=0x%08x mask=0x%08x\n", filter_count, id, mask);
//    filter_count++;
//    return 0;
//}


void cleanup_and_exit(int sig)
{
    if (verbose)
        rt_printf("Signal %d received\n", sig);
    // Disable system before closing connection
    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
    CAN_closeCan(can_handle);
    exit(0);
}

#ifdef  __XENO__
void my_task(void)
#elif defined __RT_PREEMPT__
void *my_task(void *data)
#endif
{
    int ret, count = 0, print = 4000;
    CAN_Frame_t frame;

#if defined __RT_PREEMPT__
    struct timespec now;
    struct timespec prev;
    struct timespec elapsed;
    clock_gettime(CLOCK_REALTIME, &prev);
#endif

    // configure OptoForce
    OPTO_sendConfig(can_handle,
            OPTO_CONFIG_SAMPLE_FREQ_1000,
            OPTO_CONFIG_FILTER_FREQ_15,
            OPTO_CONFIG_SET_ZERO);

    // enable board
    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);

    // configure board to send all motor data
    ret = BLMC_sendCommand(can_handle, BLMC_CMD_SEND_ALL, BLMC_ENABLE);
    if (ret < 0) {
        // TODO: Add support for error handling.
        // switch (ret) {
        //     case -ETIMEDOUT:
        //         if (verbose)
        //             rt_printf("rt_dev_send(to): timed out");
        //         break;
        //     case -EBADF:
        //         if (verbose)
        //             rt_printf("rt_dev_send(to): aborted because socket was closed");
        //         break;
        //     default:
        //         rt_fprintf(stderr, "rt_dev_send: %s\n", strerror(-ret));
        //         break;
        // }
    }

    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
    // BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_MTR2, BLMC_ENABLE);

    // Receive messages and print board status
    while (1) {
        //BLMC_sendMotorCurrent(can_handle, 0, 0.3);

        // get the next frame from the CAN bus
        ret = CAN_receiveFrame(can_handle, &frame);
        if (ret < 0) {
            // TODO: Add support for error handling.
            // switch (ret) {
            // case -ETIMEDOUT:
            //     if (verbose)
            //         rt_printf("rt_dev_recv: timed out");
            //     continue;
            // case -EBADF:
            //     if (verbose)
            //         rt_printf("rt_dev_recv: aborted because socket was closed");
            //     break;
            // default:
            //     rt_fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
            // }
            break;
        }

        // Show the frame to all involved modules. Stop as soon as one of them
        // can make use of it.
        do {
            ret = BLMC_processCanFrame(&frame, &board_data);
            if (ret != BLMC_RET_FOREIGN_FRAME)
                break;

            ret = OPTO_processCanFrame(&frame, &opto_data);
            if (ret != OPTO_RET_FOREIGN_FRAME)
                break;
        } while(0);

        if (print && (count % print) == 0) {
            rt_printf("#%d: (%d)\n", count, can_con.msg_addr.can_ifindex);
#ifdef __RT_PREEMPT__
            clock_gettime(CLOCK_REALTIME, &now);
            timespec_sub(&elapsed, &now, &prev);
            prev = now;
            float elapsed_sec = (float)(elapsed.tv_sec) + (elapsed.tv_nsec/1e9);
            rt_printf("(Receiving %d pkgs took time_sec=%0.6f, freq=%0.3f)\n",
                print, elapsed_sec, (float)(print)/elapsed_sec);
#endif

            BLMC_printSynchronizedBoardStatus(&board_data);

            if (opto_data.has_new_data) {
                opto_data.has_new_data = false;
                rt_printf("OptoForce:\n");
                rt_printf("\t(%d, %d, %d) [counts]\n", opto_data.data.fx_counts,
                        opto_data.data.fy_counts, opto_data.data.fz_counts);
                rt_printf("\t(%.2f, %.2f, %.2f) [N]\n", opto_data.data.fx_N,
                        opto_data.data.fy_N, opto_data.data.fz_N);
            }

            rt_printf("\n");
        }
        count++;
    }
}


int main(int argc, char **argv)
{
    int ret, priority;
    u_int32_t err_mask = 0;
    char name[32];

    // for memory management
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
    can_handle = CAN_initCanHandle(&can_con);
    BLMC_initBoardData(&board_data, BLMC_SYNC_ON_ADC6);
    OPTO_initOptoForceData(&opto_data, OPTO_COUNTS_AT_NC_FZ);

    if (argc <= 1) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        printf("where <can_interface> is something like 'rtcan0'.\n");
        return -1;
    }

    ret = CAN_setupCan(can_handle, argv[1], err_mask);
    if (ret < 0) {
        printf("Couldn't setup CAN connection. Exit.");
        return -1;
    }

    snprintf(name, sizeof(name), "blmc_simple_test-%d", getpid());

#ifdef __XENO__ /** Spawn a Xenomai thread **/
    // for real time printing
    rt_print_auto_init(1);

    RT_TASK task_desc; // task descriptor. Used to reference rt tasks

    priority = 10;
    // ret = rt_task_shadow(&task_desc, name, priority, 0);

    ret = rt_task_create(&task_desc, name, 0, priority,  T_JOINABLE | T_FPU);
    // rt_task_create inputs
    // task descriptor address
    // name
    // stack size. 0 defaults to "a reasonable amount"
    // priority set to 10 (doesn't really matter since we only have 1 task)
    // mode. T_JOINABLE and T_FPU are important, since we want main to wait on
    // our rt-task (see below) and FPU so we can do floating-point operations.

    if (ret) {
        fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
        CAN_closeCan(can_handle);
        return -1;
    }

    rt_task_start(&task_desc, &my_task, NULL);
    rt_task_join(&task_desc); // wait for my_task to finish before exiting main
#elif defined __RT_PREEMPT__ /** Spawn a rt_preempt thread **/
    // Based on:
    // https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base

    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;

    ret = pthread_attr_init(&attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        return ret;
    }

    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        return ret;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        return ret;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        return ret;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        return ret;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, &my_task, NULL);
    if (ret) {
        printf("create pthread failed. Ret=%d\n", ret);
        if (ret == 1) {
            printf("NOTE: This program must be executed as root to get the "
                "required realtime permissions.\n");
        }
        return ret;
    }

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread, NULL);
    if (ret)
        printf("join pthread failed.\n");
        return ret;
#endif


    return 0;
}
