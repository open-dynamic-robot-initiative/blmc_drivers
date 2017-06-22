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
#include <native/task.h>
#include <rtdk.h>
#include <blmc_can/can.h>
#include <blmc_can/blmc_can.h>
#include <blmc_can/optoforce_can.h>
#include <time.h>
#include <math.h>

#define BOARD_1 0
#define BOARD_2 1

// GLOBALS
// **************************************************************************
const static int verbose = 0;
RT_TASK rt_task_desc;
//#define MAX_FILTER 16
//struct can_filter recv_filter[MAX_FILTER];
//static int filter_count = 0;

CAN_CanConnection_t can_con[2];
CAN_CanHandle_t can_handle[2];

//! global board data
BLMC_BoardData_t board_data[2];

OPTO_OptoForceData_t opto_data;

double motor_radius = 0.015;


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
    for (int board_num = BOARD_1; board_num < 2; ++board_num)
    {
      // Disable system before closing connection
      BLMC_sendCommand(can_handle[board_num], BLMC_CMD_ENABLE_SYS, BLMC_DISABLE);
      CAN_closeCan(can_handle[board_num]);
    }
    exit(0);
}

int send_motor(double current)
{
    int ret;
    ret = BLMC_sendMotorCurrent(can_handle[BOARD_2], current, 0);
    if (ret < 0) {
        printf("ERROR: Failed to send message: current = %f\n", current);
          return -1;
    }
    return 0;
}

// **************************************************************************

void get_next_frame(CAN_Frame_t frame)
{
    int ret;
    for (int board_num = BOARD_1; board_num < 2; ++board_num)
    {
        // get the next frame from the CAN bus
        ret = CAN_receiveFrame(can_handle[board_num], &frame);
        if (ret < 0) {
            switch (ret) {
            case -ETIMEDOUT:
                if (verbose)
                    rt_printf("rt_dev_recv: timed out [board %d].\n", board_num+1);
                continue;
            case -EBADF:
                if (verbose)
                    rt_printf("rt_dev_recv: aborted because socket was closed [board %d].\n", board_num+1);
                break;
            default:
                rt_fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
            }
            break;
        }

        // Show the frame to all involved modules. Stop as soon as one of them
        // can make use of it.
        do {
            ret = BLMC_processCanFrame(&frame, &board_data[board_num]);
            if (ret != BLMC_RET_FOREIGN_FRAME)
                break;
            if (board_num == BOARD_1)
            {
                ret = OPTO_processCanFrame(&frame, &opto_data);
                if (ret != OPTO_RET_FOREIGN_FRAME)
                    break;
            }
        } while(0);
    }
}

// **************************************************************************

void print_board_status(int print, int count)
{
    for (int board_num = BOARD_1; board_num < 2; ++board_num)
    {
        if (print && (count % print) == 0) {
            rt_printf("******************* BOARD %d *******************", board_num+1);
            rt_printf("#%d: (%d)\n", count, can_con[board_num].msg_addr.can_ifindex);
            BLMC_printSynchronizedBoardStatus(&board_data[board_num]);
            rt_printf("\n");

            if (board_num == BOARD_1)
            {
                if (opto_data.has_new_data) {
                    opto_data.has_new_data = false;
                    rt_printf("OptoForce:\n");
                    rt_printf("\t(%d, %d, %d) [counts]\n", opto_data.data.fx_counts,
                            opto_data.data.fy_counts, opto_data.data.fz_counts);
                    rt_printf("\t(%.2f, %.2f, %.2f) [N]\n", opto_data.data.fx_N,
                            opto_data.data.fy_N, opto_data.data.fz_N);
                }
            }
        }
    }
}

void print_height_status(int print, int count)
{
    int board_num = BOARD_1;
    if (print && (count % print) == 0) {
        rt_printf("#%d:\n", count);
        rt_printf("Height: %f\n", board_data[board_num].sync.adc6.value[0]);
    }
}

// **************************************************************************

double initialize()
{
    int ret, priority;
    char const *can_interface[2];
    can_interface[0] = "rtcan0";
    can_interface[1] = "rtcan1";
    u_int32_t err_mask = 0;
    char name[32];

    for (int board_num = BOARD_1; board_num < 2; ++board_num) {
      can_handle[board_num] = CAN_initCanHandle(&can_con[board_num]);
      BLMC_initBoardData(&board_data[board_num], BLMC_SYNC_ON_ADC6);

      ret = CAN_setupCan(can_handle[board_num], can_interface[board_num], err_mask);
      if (ret < 0) {
          rt_printf("Couldn't setup CAN connection. Exit [board %d].\n", board_num+1);
          return -1;
      }

    }
    int board_num = BOARD_2;
    snprintf(name, sizeof(name), "blmc_simple_test-%d", getpid());
    priority = 10;
    ret = rt_task_shadow(&rt_task_desc, name, priority, 0);
    if (ret) {
        rt_fprintf(stderr, "rt_task_shadow: %s\n", strerror(-ret));
        CAN_closeCan(can_handle[board_num]);
        return -1;
    }

    int count = 0;
    CAN_Frame_t frame;

    for (int board_num = BOARD_1; board_num < 2; ++board_num)
    {
        // configure OptoForce of BOARD_1
        if (board_num == BOARD_1)
        {
          OPTO_sendConfig(can_handle[board_num],
                  OPTO_CONFIG_SAMPLE_FREQ_1000,
                  OPTO_CONFIG_FILTER_FREQ_15,
                  OPTO_CONFIG_SET_ZERO);
        }

        // enable board
        BLMC_sendCommand(can_handle[board_num], BLMC_CMD_ENABLE_SYS, BLMC_ENABLE);

        // configure board to send all motor data
        ret = BLMC_sendCommand(can_handle[board_num], BLMC_CMD_SEND_ALL, BLMC_ENABLE);
        if (ret < 0) {
            switch (ret) {
                case -ETIMEDOUT:
                    if (verbose)
                        rt_printf("rt_dev_send(to): timed out [board %d].\n", board_num+1);
                    break;
                case -EBADF:
                    if (verbose)
                        rt_printf("rt_dev_send(to): aborted because socket was closed [board %d].\n", board_num+1);
                    break;
                default:
                    rt_fprintf(stderr, "rt_dev_send: %s\n", strerror(-ret));
                    break;
            }
        }

        // before enabling motors, make sure current is set to zero.
        ret = BLMC_sendMotorCurrent(can_handle[board_num], 0, 0);
        if (ret < 0) {
          if (verbose)
              rt_printf("ERROR: Failed to send message CURRENT=0 [board %d].\n", board_num+1);
          return -1;
        }
        // enable motors
        if (verbose)
            rt_printf("Enable motor\n");
        ret = BLMC_sendCommand(can_handle[board_num], BLMC_CMD_ENABLE_MTR1, BLMC_ENABLE);
        if (ret < 0) {
          if (verbose)
              rt_printf("ERROR: Failed to send message CMD_ENABLE_MTR1 [board %d].\n", board_num+1);
          return -1;
        }
    }

    double h;

    h = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];
    while (h < 0.01)
    {
        // Receive messages
        get_next_frame(frame);
        h = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];

        count++;
        if (count > 100)
        {
          if (verbose)
              rt_printf("ERROR not receiving or position too high.\n");
          exit(1);
        }
    }
    rt_printf("Needed %d loop for initialization.\n", count);
    return 0;
}

// **************************************************************************

double remove_slack(bool verbose)
{
    if (verbose)
    {
        rt_printf("--- Remove slack.\n");
    }
    int count = 0, print = 1000;

    CAN_Frame_t frame;

    double initial_motor_position = board_data[BOARD_2].sync.position.value[0];
    double initial_height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];

    double height;
    // Receive messages and print board status
    while (count < 5000 && abs(height-initial_height) < 0.001) {
        get_next_frame(frame);
        // print_board_status(print, count);
        height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];

        send_motor(0.3);
        count++;
    }

    send_motor(0);
    double final_motor_position = board_data[BOARD_2].sync.position.value[0];
    if (verbose)
    {
        rt_printf("Slack removed. Position of motor: %f -> %f\n", initial_motor_position, final_motor_position);
          rt_printf("Initial height: %f\n", height);
    }
    return (height);
}

// **************************************************************************

int give_slack(double offset, bool verbose)
{
    if (verbose)
    {
        rt_printf("--- Give slack (%f).\n", offset);
    }
    int count = 0, print = 5000;
    CAN_Frame_t frame;

    double motor_value = (offset/(2*M_PI*motor_radius))*9;
    if (verbose)
    {
        rt_printf("motor_value: %f\n", motor_value);
    }

    double initial_motor_position = board_data[BOARD_2].sync.position.value[0];
    double motor_position = board_data[BOARD_2].sync.position.value[0];

    // Receive messages and print board status
    while (initial_motor_position - motor_position < motor_value)
    {
        get_next_frame(frame);
        motor_position = board_data[BOARD_2].sync.position.value[0];
        count++;
        send_motor(-0.3);
    }
    send_motor(0);
    if (verbose)
    {
        rt_printf("Slack retrieved.\n");
    }

}

// **************************************************************************

int go_up(double final_height, bool verbose)
{
    CAN_Frame_t frame;
    if (verbose)
    {
        rt_printf("--- Go up until %f.\n", final_height);
    }
    int ret, count = 0, print = 100;

    double current, current_max;
    current = 0.0;
    current_max = 0.6;

    // Compute heights to go up
    get_next_frame(frame);
    double initial_height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];
    double height_to_climb = initial_height - final_height;

    double h_0 = initial_height;
    double h_1 = initial_height - height_to_climb/3;
    double h_2 = initial_height - 2*height_to_climb/3;
    double h_3 = final_height;

    if (verbose)
    {
        rt_printf("Initial height: %f\n", initial_height);
        rt_printf("height_to_climb: %f\n", height_to_climb);
        rt_printf("h0: %f, h1: %f, h2: %f, h3: %f\n", h_0, h_1, h_2, h_3);
    }

    // Receive messages and print board status
    double height = initial_height;
    while (1) {
        get_next_frame(frame);
        if (verbose)
        {
            print_height_status(print, count);
        }
        // print_board_status(print, count);

        height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];
        // check if current is not too high
        if (current < current_max & current > -current_max)
        {
          if (h_0 >= height && height > h_1)
          {
              send_motor(0.8);
          }
          if (h_1 >= height && height > h_2)
          {
              send_motor(0.6);
          }
          if (h_2 >= height && height > h_3)
          {
              send_motor(0.4);
          }
          if (h_3 >= height)
          {
              send_motor(0);
              break;
          }
        }
        else
        {
            send_motor(0);
        }
        count++;
    }
    send_motor(0);
    if (verbose)
    {
        rt_printf("height: %f\n", height);
    }
}

// **************************************************************************

int stabilize(int wait_time, double motor_current, bool verbose)
{
    int count = 0, print = 100;
    CAN_Frame_t frame;
    int t0 = time(NULL);
    int t1 = time(NULL);
    if (verbose)
    {
        rt_printf("--- Stabilize for : %d sec\n", wait_time);
    }

    // Receive messages and print board status
    while (t1 - t0 < wait_time) {
        get_next_frame(frame);
        if (verbose)
        {
            print_height_status(print, count);
        }
        send_motor(motor_current);
        t1 = time(NULL);
        count++;
    }
    send_motor(0);
    return(0);
}

// **************************************************************************

int read_values()
{
    int ret, count = 0, print = 100;
    CAN_Frame_t frame;
    // Receive messages and print board status
    while (1) {
        get_next_frame(frame);
        print_height_status(print, count);
        count++;
    }
}

// **************************************************************************


int go_down(double final_height, bool verbose)
{
    CAN_Frame_t frame;
    if (verbose)
    {
        rt_printf("--- Go down until %f.\n", final_height);
    }
    int ret, count = 0, print = 100;

    double current, current_max;
    current = 0.0;
    current_max = 0.1;

    // Compute heights to go up
    get_next_frame(frame);
    double initial_height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];

    double height_to_climb = initial_height - final_height;

    double h_0 = initial_height;
    double h_1 = initial_height - height_to_climb/3;
    double h_2 = initial_height - 2*height_to_climb/3;
    double h_3 = final_height;

    if (verbose)
    {
        rt_printf("Initial height: %f\n", initial_height);
        rt_printf("height_to_climb: %f\n", height_to_climb);
        rt_printf("h0: %f, h1: %f, h2: %f, h3: %f\n", h_0, h_1, h_2, h_3);
    }

    // Receive messages and print board status
    double height = initial_height;
    while (1) {
        get_next_frame(frame);
        if (verbose)
        {
            print_height_status(print, count);
        }
        // print_board_status(print, count);

        height = board_data[BOARD_1].sync.adc6.value[BLMC_ADC_A];
        // check if current is not too high
        // if (current < current_max & current > -current_max)
        // {
        //   if (h_0 <= height && height < h_1)
        //   {
        //       send_motor(0);
        //   }
        //   if (h_1 <= height && height < h_2)
        //   {
        //       send_motor(0);
        //   }
        //   if (h_2 <= height && height < h_3)
        //   {
        //       send_motor(0);
        //   }
        //   if (h_3 <= height)
        //   {
        //       send_motor(0);
        //       break;
        //   }
        // }
        // else
        // {
        //     send_motor(0);
        // }
        if (height < final_height)
        {
          send_motor(0);
        }
        else
        {
          send_motor(0);
          break;
        }
        count++;
    }
    send_motor(0);
    if (verbose)
    {
        rt_printf("height: %f\n", height);
    }

}


// **************************************************************************


int main(int argc, char **argv)
{
    // for memory management
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // for real time printing
    rt_print_auto_init(1);

    signal(SIGTERM, cleanup_and_exit);
    signal(SIGINT, cleanup_and_exit);

    // Initialize stuff
    // ----------------
    //

    double initial_height;
    initialize();

    int count=0;
    while (1)
    {
      rt_printf("Trial %d\n", count);
      initial_height = remove_slack(false);
      go_up(0.41, false);
      stabilize(2, 0.3, false);
      double final_height = initial_height-0.01;
      go_down(final_height, false);
      stabilize(2, 0.3, false);
      give_slack(0.1, false);
      sleep(2);
      count++;
    }
    return 0;
}
