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


#ifndef BLMC_CAN_MOTOR_CLASS_CPP_
#define BLMC_CAN_MOTOR_CLASS_CPP_



#include <stdlib.h>
#include <rtdk.h>
#include <math.h>

#include "blmc_can_motor_class.h"


// **************************************************************************

LifterReset::LifterReset(double motor_radius)
{
  motor_radius_ = motor_radius;
  lift_state_ = LiftState::UNINITIALIZED;
}

// **************************************************************************

void LifterReset::update_motor_position(double new_motor_position)
{
  motor_position_ = new_motor_position;
}

void LifterReset::update_height(double new_height)
{
  height_ = new_height;
}

// **************************************************************************


void LifterReset::print_readings()
{
  rt_printf("Height: %f\n", height_);
  rt_printf("Motor position: %f\n", motor_position_);
}


// **************************************************************************

LiftState LifterReset::get_state()
{
  return lift_state_;
}

// **************************************************************************

double LifterReset::get_torque()
{
    switch (lift_state_) {
      case LiftState::UNINITIALIZED:
        rt_printf("Lift state: UNINITIALIZED\n");
        return 0; // Should only get here when the task was initialized.

      case LiftState::IDLE:
        rt_printf("Lift state: IDLE\n");
        return 0;

      case LiftState::GOING_UP:
      {
        double torque = go_up_();
        // rt_printf("Lift state: GOING_UP (%f)\n", torque);
        return torque;
      }

      case LiftState::GOING_DOWN:
      {
        double torque = go_down_();
        // rt_printf("Lift state: GOING_DOWN (%f)\n", torque);
        return torque;
      }
      case LiftState::ADDING_SLACK:
        // rt_printf("Lift state: ADDING_SLACK\n");
        return add_slack_();

      case LiftState::REMOVING_SLACK:
        // rt_printf("Lift state: REMOVING_SLACK\n");
        return remove_slack_();

      default:
        rt_printf("Lift state: ERROR\n");
        exit(1);
    }
}

// **************************************************************************

void LifterReset::go_up(double final_height)
{
    initial_height_ = height_;
    final_height_ = final_height;

    double height_to_climb = initial_height_ - final_height_;

    h_0_ = initial_height_;
    h_1_ = initial_height_ - height_to_climb/3;
    h_2_ = initial_height_ - 2*height_to_climb/3;
    h_3_ = final_height_;

    if (verbose)
    {
        rt_printf("Initial height: %f\n", initial_height_);
        rt_printf("height_to_climb: %f\n", height_to_climb);
        rt_printf("h0: %f, h1: %f, h2: %f, h3: %f\n", h_0_, h_1_, h_2_, h_3_);
    }
    lift_state_ = LiftState::GOING_UP;

}

double LifterReset::go_up_()
{
    double torque_to_apply_;
    if (h_0_ >= height_ && height_ > h_1_)
    {
        torque_to_apply_ = 0.8;
    }
    if (h_1_ >= height_ && height_ > h_2_)
    {
        torque_to_apply_ = 0.6;
    }
    if (h_2_ >= height_ && height_ > h_3_)
    {
        torque_to_apply_ = 0.4;
    }
    if (h_3_ >= height_)
    {
        torque_to_apply_ = 0.3;
        lift_state_ = LiftState::IDLE;
    }

    if (torque_to_apply_ > torque_max)
    {
        if (verbose)
        {
            rt_printf("Torques to apply is too high (%f > %f)\n", torque_to_apply_, torque_max);
        }
        torque_to_apply_ = 0;
    }
    return torque_to_apply_;
}

// **************************************************************************

void LifterReset::go_down(double final_height)
{
    initial_height_ = height_;
    final_height_ = final_height;

    double height_to_climb = initial_height_ - final_height_;

    if (verbose)
    {
        rt_printf("Initial height: %f\n", initial_height_);
        rt_printf("Final height: %f\n", final_height_);
    }
    lift_state_ = LiftState::GOING_DOWN;

}

double LifterReset::go_down_()
{
    double torque_to_apply_;
    if (height_ < final_height_)
    {
      torque_to_apply_ = 0;
    }
    else
    {
      torque_to_apply_ = 0.3;
      lift_state_ = LiftState::IDLE;
    }
    return torque_to_apply_;
}


// **************************************************************************

void LifterReset::remove_slack()
{
    remove_slack_timeout_ = 0;
    lift_state_ = LiftState::REMOVING_SLACK;
}

double LifterReset::remove_slack_()
{
    initial_height_ = height_;
    double torque_to_apply_;

    if (remove_slack_timeout_ < 5000 && abs(height_-initial_height_) < 0.001)
    {
        torque_to_apply_ = 0.3;
        remove_slack_timeout_++;
    }
    else
    {
        torque_to_apply_ = 0;
        lift_state_ = LiftState::IDLE;
    }
    return torque_to_apply_;
}

// **************************************************************************

void LifterReset::add_slack(double slack_offset)
{
    motor_offset_ = (slack_offset/(2*M_PI*motor_radius_))*9;
    initial_motor_position_ = motor_position_;
    lift_state_ = LiftState::ADDING_SLACK;
}

double LifterReset::add_slack_()
{
    double torque_to_apply_;

    if (initial_motor_position_ - motor_position_ < motor_offset_)
    {
        torque_to_apply_ = -0.3;
    }
    else
    {
        torque_to_apply_ = 0;
        lift_state_ = LiftState::IDLE;
    }
    return torque_to_apply_;
}


int main_lifter_reset_class(int argc, char **argv)
{
    // for real time printing
    rt_print_auto_init(1);

    LifterReset lifter_reset(0.015);
    lifter_reset.verbose = true;
    lifter_reset.update_height(0.61);
    lifter_reset.print_readings();
    lifter_reset.torque_max = 1.0;

    lifter_reset.go_up(0.41);
    double torque_to_apply_ = lifter_reset.get_torque();
    rt_printf("torque to apply: %f\n", torque_to_apply_);
    return 0;
}

#endif
