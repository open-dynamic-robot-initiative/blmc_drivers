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
#endif
#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>
#include <blmc_can/optoforce_can.h>


// GLOBALS
// **************************************************************************

const static int verbose = 0;
RT_TASK rt_task_desc;
//#define MAX_FILTER 16
//struct can_filter recv_filter[MAX_FILTER];
//static int filter_count = 0;

CAN_CanConnection_t can_con;
CAN_CanHandle_t can_handle;

//! global board data
BLMC_BoardData_t board_data;

OPTO_OptoForceData_t opto_data;

uint16_t sample_counter_offset = 0;
bool first_recorded_package = true;


// FUNCTIONS
// **************************************************************************

void cleanup_and_exit(int sig)
{
    if (verbose)
        rt_printf("Signal %d received\n", sig);
    // Disable system before closing connection
    BLMC_sendCommand(can_handle, BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
    CAN_closeCan(can_handle);
    exit(0);
}


void rt_task(void)
{
    int ret, count = 0, print = 1000, counter_new_pkg = 0;
    bool print_new_data = false;
    CAN_Frame_t frame;

    FILE *log_file = fopen("/tmp/blmc_can_demo_dumper.csv","w");
    rt_printf("Logging data at: /tmp/blmc_can_demo_dumper.csv\n");

    fprintf(log_file, "# time[ms], count,opto_data.data.sample_counter, "
        "opto_data.data.fx_counts,opto_data.data.fy_counts,opto_data.data.fz_counts,"
        "opto_data.data.fx_N,opto_data.data.fy_N,opto_data.data.fz_N\n");

    rt_printf("Setup optoforce sensor...\n");

    // configure OptoForce
    OPTO_sendConfig(can_handle,
            OPTO_CONFIG_SAMPLE_FREQ_1000,
            OPTO_CONFIG_FILTER_FREQ_15,
            OPTO_CONFIG_SET_ZERO);

    rt_printf("Start main can frame processing loop...\n");

    // Receive messages and print board status
    while (1) {
        //BLMC_sendMotorCurrent(can_handle, 0, 0.3);

        // get the next frame from the CAN bus
        ret = CAN_receiveFrame(can_handle, &frame);
        if (ret < 0) {
            switch (ret) {
            case -ETIMEDOUT:
                if (verbose)
                    rt_printf("rt_dev_recv: timed out");
                continue;
            case -EBADF:
                if (verbose)
                    rt_printf("rt_dev_recv: aborted because socket was closed");
                break;
            default:
                rt_fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
            }
            break;
        }

        // Show the frame to all involved modules. Stop as soon as one of them
        // can make use of it.
        do {
            ret = OPTO_processCanFrame(&frame, &opto_data);
            if (ret != OPTO_RET_FOREIGN_FRAME)
                break;
        } while(0);

        if (print && (count % print) == 0) {
           print_new_data = true;
        }

        if (opto_data.has_new_data) {
            opto_data.has_new_data = false;

            if (first_recorded_package == true) {
                first_recorded_package = false;
                sample_counter_offset = opto_data.data.sample_counter;
            }

            fprintf(log_file, "%0.3f,%d,%d,%d,%d,%d,%0.2f,%0.2f,%0.2f\n",
                // Convert to microseconds.
                // See: https://stackoverflow.com/a/12948865
                ((double)rt_timer_read()) / 1000.,
                count,

                // Substracting the initial sample_counter value to avoid
                // overflows while recording the data.
                opto_data.data.sample_counter - sample_counter_offset,

                opto_data.data.fx_counts,
                opto_data.data.fy_counts,
                opto_data.data.fz_counts,
                opto_data.data.fx_N,
                opto_data.data.fy_N,
                opto_data.data.fz_N);


            if (print_new_data) {
                print_new_data = false;

                rt_printf("#%d: (%d)\n", count, can_con.msg_addr.can_ifindex);

                rt_printf("OptoForce:\n");
                rt_printf("\t(%d, %d, %d) [counts]\n", opto_data.data.fx_counts,
                        opto_data.data.fy_counts, opto_data.data.fz_counts);
                rt_printf("\t(%.2f, %.2f, %.2f) [N]\n", opto_data.data.fx_N,
                        opto_data.data.fy_N, opto_data.data.fz_N);

                rt_printf("\n");
            }
        }
        count++;
    }
}


int main(int argc, char **argv)
{
    int ret, priority;
    u_int32_t err_mask = 0;
    char name[32];

    mlockall(MCL_CURRENT | MCL_FUTURE);

    // for real time printing
    rt_print_auto_init(1);

    signal(SIGTERM, cleanup_and_exit);
    signal(SIGINT, cleanup_and_exit);

    // Initialize stuff
    // ----------------
    //
    can_handle = CAN_initCanHandle(&can_con);
    // BLMC_initBoardData(&board_data, BLMC_SYNC_ON_ADC6);
    OPTO_initOptoForceData(&opto_data, OPTO_COUNTS_AT_NC_FZ);

    if (argc <= 1) {
        rt_printf("Usage: %s <can_interface>\n", argv[0]);
        rt_printf("where <can_interface> is something like 'rtcan0'.\n");
        return -1;
    }

    ret = CAN_setupCan(can_handle, argv[1], err_mask);
    if (ret < 0) {
        rt_printf("Couldn't setup CAN connection. Exit.");
        return -1;
    }

    snprintf(name, sizeof(name), "blmc_simple_test-%d", getpid());
    priority = 10;
    ret = rt_task_shadow(&rt_task_desc, name, priority, 0);
    if (ret) {
        rt_fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
        CAN_closeCan(can_handle);
        return -1;
    }
    rt_task(); /* never returns */

    return 0;
}
