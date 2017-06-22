
/**
 * \class MotorCurrentTracker
 *
 * \brief Class to record and track the maximum current applied to a motor.
 *
 * The average current is tracked over a time window. If the averaged current
 * within this window exceeds a limit once, then the 'record_current' method
 * returns `false` always.
 */

#ifndef BLMC_CAN_MOTOR_CLASS_H_
#define BLMC_CAN_MOTOR_CLASS_H_


enum class LiftState {
  UNINITIALIZED = 0,
  IDLE,
  GOING_UP,
  GOING_DOWN,
  GOING_UP_HOLD,
  GOING_DOWN_HOLD,
  ADDING_SLACK,
  REMOVING_SLACK,
};

class LifterReset
{
  public:
    LifterReset(double motor_radius);
    void update_motor_position(double new_motor_position);
    void update_height(double new_height);
    void print_readings();
    double get_torque();
    LiftState get_state();
    void go_up(double final_height);
    void go_down(double final_height);
    void remove_slack();
    void add_slack(double slack_offset);
    bool verbose;
    double torque_max;

  private:
    double motor_radius_;
    double height_;
    double motor_position_;
    LiftState lift_state_;

    // Going up and down
    double go_up_();
    double go_down_();
    double final_height_;
    double initial_height_;
    double h_0_;
    double h_1_;
    double h_2_;
    double h_3_;

    double remove_slack_();
    double add_slack_();
    int remove_slack_timeout_;
    double motor_offset_;
    double initial_motor_position_;

};

#endif
