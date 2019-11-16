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
     * 
     * The constructor starts a real time thread: EthernetWifiMotorBoard::loop().
     * This thread streams the data back and forth collecting the sensor data
     * and sends the control/commands.
     * 
     * @param master_board_interface is the object that communicate with the
     * master board. The master board provides the hardware informations.
     * @param slave_id is the id of the udriver this class represents.
     * @param history_length is the size of the buffer of messages stored.
     */
    EthernetWifiMotorBoard(
      std::shared_ptr<MasterBoardInterface> master_board_interface,
      const size_t slave_id, const size_t history_length=1000);

    /**
     * @brief Destroy the EthernetWifiMotorBoard object. The destructor handles
     * the proper shutdown of the class and the threads.
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
     * Inherited from MotorBoardInterface. This particualr instance does not
     * actually check if it is is a new command or control as the full status of
     * the robot is exchange at every tick.
     */
    virtual void send_if_input_changed();

    /**
     * @brief return s only once board and motors are ready.
     */
    bool is_ready();

private:
    /**
     * Private methods
     */

    /**
     * @brief This is the helper function used for spawning the real time
     * thread.
     *
     * @param instance_pointer is the current object in this case.
     * @return THREAD_FUNCTION_RETURN_TYPE depends on the current OS.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        static_cast<EthernetWifiMotorBoard*>(instance_pointer)->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief This is the real time thread that streams the data to/from the
     * master board. 
     */
    void loop();

    /**
     * Communication related attributes
     */

    /**
     * @brief Master board interface sdk:
     * https://github.com/open-dynamic-robot-initiative/master-board
     */
    std::shared_ptr<MasterBoardInterface> master_board_interface_;

    /**
     * @brief This is the driver of the udriver. It allows to communicate with
     * the motors.
     */
    MotorDriver* udriver_;

    /**
     * @brief slave_id_ is the index of the udriver (slave) controlled by the
     * master board. The index is the one used in MasterBoardInterface. The
     * index should correspond to the hardware SPI index onboard the card.
     */
    size_t slave_id_;

    /**
     * Outputs
     */

    /**
     * @brief measurement_ contains all the measurements acquiered from the CAN
     * board.
     */
    std::vector<std::shared_ptr<ScalarTimeseries> > measurement_;

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
    std::vector<std::shared_ptr<ScalarTimeseries> > control_;

    /**
     * @brief This is the buffer of the commands to be sent to the card.
     */
    std::shared_ptr<CommandTimeseries> command_;

    /*! @brief history_length_ is the length of data buffers in number of
    iteration. */
    size_t history_length_;

    /**
     * Log
     */

    /**
     * @brief This is the history of the already sent controls.
     */
    std::vector<std::shared_ptr<ScalarTimeseries> > sent_control_;

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

    /**
     * @brief Are motor in idle mode = 0 torques?
     */
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