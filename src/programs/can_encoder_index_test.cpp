/**
 * \file
 * \brief Simple test tool to check if the encoder index is detected.
 *
 * Runs a motor at a constant torque and prints a line whenever the encoder
 * index is detected.
 *
 * \copyright Copyright (c) 2020 Max Planck Gesellschaft.
 */
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>

#include <blmc_drivers/blmc_joint_module.hpp>

using namespace blmc_drivers;

class EncoderIndexTester
{
public:
    static constexpr double TORQUE_CONSTANT_NmpA = 0.02;
    static constexpr double GEAR_RATIO = 9.0;
    static constexpr double MAX_CURRENT_A = 2.0;
    static constexpr double ONE_MOTOR_ROTATION_DISTANCE =
        2.0 * M_PI / GEAR_RATIO;

    //! The max. mrev position of a joint (i.e. the point where the position
    //! measurement of the board rolls over.
    static constexpr double MAX_MOTOR_POSITION_MREV = 100;
    //! The max. position transformed to joint radian.
    static constexpr double MAX_JOINT_POSITION_RAD =
        MAX_MOTOR_POSITION_MREV * 2 * M_PI / GEAR_RATIO;

    EncoderIndexTester(const std::string &can_port,
                       int motor_index,
                       double torque,
                       unsigned int number_of_revolutions)
        : torque_(torque), number_of_revolutions_(number_of_revolutions)
    {
        std::cout << "CAN port: " << can_port << std::endl;
        std::cout << "Motor Index: " << motor_index << std::endl;
        std::cout << "Torque: " << torque << std::endl;

        std::cout << "==============================" << std::endl;
        std::cout << "Initialising..." << std::endl;

        // setup can bus
        auto can_bus = std::make_shared<CanBus>(can_port);

        // set up motor board
        motor_board_ = std::make_shared<CanBusMotorBoard>(can_bus, 1000, 10);
        motor_board_->wait_until_ready();

        std::shared_ptr<MotorInterface> motor =
            std::make_shared<Motor>(motor_board_, motor_index);

        joint_module_ = std::make_unique<BlmcJointModule>(
            motor, TORQUE_CONSTANT_NmpA, GEAR_RATIO, 0, false, MAX_CURRENT_A);
    }

    void run_print_occurences()
    {
        std::cout << "Start moving with constant torque" << std::endl;
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);
        double last_index_position = joint_module_->get_measured_index_angle();

        // Do not print, rotate 100 times, count index ticks
        // check after each rotation if tick was found

        while (true)
        {
            joint_module_->set_torque(torque_);
            joint_module_->send_torque();

            double index_position = joint_module_->get_measured_index_angle();

            if (!std::isnan(index_position) &&
                index_position != last_index_position)
            {
                double diff = index_position - last_index_position;
                std::cout << "Found Encoder Index"
                          << ".\tPosition: " << index_position
                          << ".\tDiff to last: " << diff << std::endl;

                last_index_position = index_position;
            }
            spinner.spin();
        }
    }

    void run_verify_occurences()
    {
        std::cout << "Start moving with constant torque" << std::endl;
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);

        double last_index_position = joint_module_->get_measured_index_angle();
        unsigned int counter = 0;

        double initial_position = joint_module_->get_measured_angle();
        double move_distance =
            number_of_revolutions_ * ONE_MOTOR_ROTATION_DISTANCE;
        double target_position = initial_position + move_distance;

        bool has_error = false;

        // Do not print, rotate N times, count index ticks
        // check after each rotation if tick was found
        while (rollover_safe_distance(initial_position,
                                      joint_module_->get_measured_angle()) <
               move_distance)
        {
            // check for board errors
            auto board_status = motor_board_->get_status()->newest_element();
            if (board_status.error_code != MotorBoardStatus::ErrorCodes::NONE)
            {
                std::cout << "ERROR: " << board_status.get_error_description()
                          << std::endl;
                has_error = true;
                break;
            }

            joint_module_->set_torque(torque_);
            joint_module_->send_torque();

            double index_position = joint_module_->get_measured_index_angle();

            if (!std::isnan(index_position) &&
                index_position != last_index_position)
            {
                counter++;
                last_index_position = index_position;
            }
            spinner.spin();
        }

        if (!has_error)
        {
            std::cout << "Finished " << number_of_revolutions_
                      << " revolutions." << std::endl;
            std::cout << "Observed encoder index tick " << counter << " times."
                      << std::endl;
        }
    }

private:
    double torque_;
    unsigned int number_of_revolutions_;
    std::shared_ptr<CanBusMotorBoard> motor_board_;
    std::unique_ptr<BlmcJointModule> joint_module_;

    /**
     * @brief Compute distance between positions with rollover-compensation.
     *
     * The result is only correct under the following constraints:
     *
     *  - The difference is positive, i.e. p1 < p2 without rollover.
     *  - There is at most one rollover between p1 and p2.
     *
     * @param p1 Joint angle [rad] of the lower position.
     * @param p2 Joint angle [rad] of the higher position.
     *
     * @return Distance between p1 and p2 assuming the above constraints are
     *         fulfilled.
     */
    double rollover_safe_distance(double p1, double p2)
    {
        // p2 is expected to be greater than p1.  If this is not the case, there
        // was a rollover
        if (p2 < p1)
        {
            p2 += 2 * MAX_JOINT_POSITION_RAD;
        }
        double diff = p2 - p1;

        return diff;
    }
};

int main(int argc, char *argv[])
{
    if (argc != 4 && argc != 5)
    {
        std::cout << "Invalid number of arguments." << std::endl;
        std::cout
            << "Usage: " << argv[0]
            << " <can port> <motor index> <torque> [<number of revolutions>]"
            << std::endl;
        return 1;
    }

    std::string can_port = argv[1];
    int motor_index = std::stoi(argv[2]);
    double torque = std::stod(argv[3]);
    int num_revolutions = (argc == 5) ? std::stoi(argv[4]) : 0;

    if (motor_index != 0 && motor_index != 1)
    {
        std::cout << "Invalid motor index.  Only '0' and '1' are allowed."
                  << std::endl;
        return 1;
    }

    constexpr int num_revolutions_limit =
        2 * EncoderIndexTester::MAX_MOTOR_POSITION_MREV;
    if (num_revolutions < 0)
    {
        std::cout << "Invalid Input: Number of revolutions has to be positive."
                  << std::endl;
        return 1;
    }
    else if (num_revolutions >= num_revolutions_limit)
    {
        std::cout << "Invalid Input: Number of revolutions has to be < "
                  << num_revolutions_limit << std::endl;
        return 1;
    }

    EncoderIndexTester tester(can_port, motor_index, torque, num_revolutions);

    real_time_tools::RealTimeThread thread;

    // If no number of revolutions are specified, use the "print occurrences"
    // mode.  Otherwise use "verify occurrences" mode.
    if (num_revolutions == 0)
    {
        thread.create_realtime_thread(
            [](void *instance_pointer) {
                ((EncoderIndexTester *)(instance_pointer))
                    ->run_print_occurences();
                return (void *)nullptr;
            },
            &tester);
    }
    else
    {
        thread.create_realtime_thread(
            [](void *instance_pointer) {
                ((EncoderIndexTester *)(instance_pointer))
                    ->run_verify_occurences();
                return (void *)nullptr;
            },
            &tester);
    }
    thread.join();

    return 0;
}
