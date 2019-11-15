/**
 * @file ethernet_wifi_motor_board.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Interface for the master board designed by Thomas Floyols
 * https://github.com/open-dynamic-robot-initiative/master-board
 * @version 0.1
 * @date 2019-11-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "master_board_sdk/master_board_interface.h"
#include "real_time_tools/thread.hpp"
#include "real_time_tools/threadsafe/threadsafe_timeseries.hpp"

#include "blmc_drivers/devices/motor_board.hpp"

namespace blmc_drivers
{

class EthernetWifiMotorBoard: public MotorBoardInterface
{

public:
    /**
     * @brief Construct a new EthernetWifiMotorBoard object.
     * 
     * The constructor initializes the connexion with the
     * master board and start a real time thread: EthernetWifiMotorBoard::loop()
     * This thread streams the data back and forth collecting the sensor data
     * and sening the control/commands.
     * 
     * @param network_id Full name one can find on the left cloumn
     * with the bash command ifconfig.
     * @param n_slaves_controlled is the number of udriver plug to master
     * board via SPI connexion. Currently (15/11/2019) one udriver can control
     * 2 motors.
     */
    EthernetWifiMotorBoard(const std::string& network_id,
                           const int n_slaves_controlled);
    /**
     * @brief Destroy the EthernetWifiMotorBoard object
     */
    ~EthernetWifiMotorBoard();

    /**
     * Output and status
     */

    /**
     * @copydoc MotorBoardInterface::get_measurement()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const ScalarTimeseries> get_measurement(
      const int& index) const ;

    /**
     * @copydoc MotorBoardInterface::get_status()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const StatusTimeseries> get_status() const;

    /**
     * input logs
     */

    /**
     * @copydoc MotorBoardInterface::get_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const ScalarTimeseries> get_control(const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const CommandTimeseries> get_command() const;

    /**
     * @copydoc MotorBoardInterface::get_sent_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_sent_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const CommandTimeseries> get_sent_command() const;

    /**
     * Setters
     */
    
    /**
     * @copydoc MotorBoardInterface::set_control()
     * Inherited from MotorBoardInterface
     */
    virtual void set_control(const double& control, const int& index);
    
    /**
     * @copydoc MotorBoardInterface::set_command()
     * Inherited from MotorBoardInterface
     */
    virtual void set_command(const MotorBoardCommand& command);

    /**
     * @copydoc MotorBoardInterface::send_if_input_changed()
     * Inherited from MotorBoardInterface
     */
    virtual void send_if_input_changed();

private:
    /**
     * Private methods
     */

    /**
     * @brief This is the real time thread that streams the data to/from the
     * master board.
     * 
     * @return THREAD_FUNCTION_RETURN_TYPE 
     */
    THREAD_FUNCTION_RETURN_TYPE loop();

    /**
     * Communication related attributes
     */

    /**
     * @brief Master board interface sdk:
     * https://github.com/open-dynamic-robot-initiative/master-board
     */
    MasterBoardInterface master_board_interface_;

    /**
     * @brief Full name one can find on the left cloumn
     * with the bash command ifconfig.
     */
    std::string network_id_;

    /**
     * @brief Number of udriver the master is controlling [0-6]. Here a slave
     * is a control card not a motor. For the blmc robots 1 slaves has 2 motors.
     */
    int n_slaves_controlled_;

    /**
     * Outputs
     */

    /**
     * @brief measurement_ contains all the measurements acquiered from the CAN
     * board.
     */
    std::vector<std::shared_ptr<ScalarTimeseries>> measurement_;

    /**
     * @brief This is the status history of the CAN board.
     */
    std::shared_ptr<StatusTimeseries> status_;

    /**
     * Inputs
     */

    /**
     * @brief This is the buffer of the controls to be sent to card.
     */
    std::vector<std::shared_ptr<ScalarTimeseries>> control_;

    /**
     * @brief This is the buffer of the commands to be sent to the card.
     */
    std::shared_ptr<CommandTimeseries> command_;

    /**
     * Log
     */

    /**
     * @brief This is the history of the already sent controls.
     */
    std::vector<std::shared_ptr<ScalarTimeseries>> sent_control_;

    /**
     * @brief This is the history of the already sent commands.
     */
    std::shared_ptr<CommandTimeseries> sent_command_;

    /**
     * Loop management
     */

    /**
     * @brief This boolean makes sure that the loop is stopped upon destruction
     * of this object.
     */
    bool is_loop_active_;


    bool motors_are_paused_;


    /**
     * @brief If no control is sent for more than control_timeout_ms_ the board
     * will shut down
     */
    int control_timeout_ms_;

    /**
     * @brief This is the thread object that allow to spwan a real-time thread
     * or not dependening on the current OS.
     */
    real_time_tools::RealTimeThread rt_thread_;
};

}