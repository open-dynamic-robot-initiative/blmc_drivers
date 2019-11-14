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
     * @brief Construct a new EthernetWifiMotorBoard object
     */
    EthernetWifiMotorBoard();
    /**
     * @brief Destroy the EthernetWifiMotorBoard object
     */
    ~EthernetWifiMotorBoard();

    /**
     * @brief This method start the initialize the connexion with the
     * master board and start a real time thread. This thread  streams the data
     * back and forth collecting the sensor data and sening the
     * control/commands.
     * 
     * @param network_id Full name one can find on the left cloumn
     * with the bash command ifconfig.
     */
    void initialize(const std::string& network_id);

    /**
     * @copydoc MotorBoardInterface::get_measurement()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const ScalarTimeseries> get_measurement(const int& index);

    /**
     * @copydoc MotorBoardInterface::get_status()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const StatusTimeseries> get_status() const;

    /**
     * input logs
     */

    /**
     * @copydoc MotorBoardInterface::get_control()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const ScalarTimeseries> get_control(const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_command()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const CommandTimeseries> get_command() const;

    /**
     * @copydoc MotorBoardInterface::get_sent_control()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const ScalarTimeseries> get_sent_control(
            const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_sent_command()
     * Inherited from MotorBoardInterface
     */
    virtual Ptr<const CommandTimeseries> get_sent_command() const;

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
     * Private methods and attributes
     */

    /**
     * @brief This is the real time thread that streams the data to/from the
     * master board.
     * 
     * @return THREAD_FUNCTION_RETURN_TYPE 
     */
    THREAD_FUNCTION_RETURN_TYPE loop();

    /**
     * @brief Master board interface sdk:
     * https://github.com/open-dynamic-robot-initiative/master-board
     */
    std::shared_ptr<MasterBoardInterface> master_board_interface_;

    /**
     * 
     */
    std::string network_id_;
};

}