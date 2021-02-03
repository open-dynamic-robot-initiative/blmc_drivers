/**
 * @file spi_bus.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Interface for the main board designed by Thomas Floyols
 * https://github.com/open-dynamic-robot-initiative/master-board
 * @version 0.1
 * @date 2019-11-14
 * 
 * @copyright Copyright (c) 2019-2020, New York University and Max Planck Gesellschaft.
 * 
 */

#pragma once

#include "master_board_sdk/master_board_interface.h"
#include "blmc_drivers/devices/motor.hpp"
#include "real_time_tools/thread.hpp"

#include "blmc_drivers/devices/motor_board.hpp"

namespace blmc_drivers
{

class SpiBus: public DeviceInterface
{

public:
    /**
     * @brief Construct a new SpiBus object
     * 
     * The constructor starts a real time thread: SpiBus::loop().
     * This thread streams the data back and forth collecting the sensor data
     * and sends the control/commands.
     * 
     * @param main_board_interface is the object that communicate with the
     * main board. The main board provides the hardware informations.
     * @param nb_udrivers is the number udrivers plugged on the main board.
     * @param history_length is the size of the buffer of messages stored.
     */
    SpiBus(
      std::shared_ptr<MasterBoardInterface> main_board_interface,
      const size_t& nb_udrivers, const size_t& history_length=1000);

    /**
     * @brief Destroy the SpiBus object. The destructor handles
     * the proper shutdown of the class and the threads.
     */
    ~SpiBus();

    /**
     * Output and status
     */

    /**
     * @brief Get the measurements from the main board.
     * 
     * @param udriver_id is the index of the spi port on the control board
     * @param index 
     * @return std::shared_ptr<const MotorInterface::ScalarTimeseries> 
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries>
    get_measurement(
      const size_t udriver_id,
      const MotorBoardInterface::MeasurementIndex& index) const ;

    /**
     * @copydoc MotorBoardInterface::get_status()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::StatusTimeseries>
    get_status(const size_t udriver_id) const;

    /**
     * input logs
     */

    /**
     * @copydoc MotorBoardInterface::get_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries> get_control(
      const size_t udriver_id,
      const MotorBoardInterface::ControlIndex& index) const;

    /**
     * @copydoc MotorBoardInterface::get_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
    get_command(const size_t udriver_id) const;

    /**
     * @copydoc MotorBoardInterface::get_sent_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries>
    get_sent_control(
        const size_t udriver_id,
        const MotorBoardInterface::ControlIndex& index) const;

    /**
     * @copydoc MotorBoardInterface::get_sent_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
    get_sent_command(const size_t udriver_id) const;

    /**
     * Setters
     */
    
    /**
     * @copydoc MotorBoardInterface::set_control()
     * Inherited from MotorBoardInterface
     */
    virtual void set_control(const size_t udriver_id, const double& control,
                             const MotorBoardInterface::ControlIndex& index);
    
    /**
     * @copydoc MotorBoardInterface::set_command()
     * Inherited from MotorBoardInterface
     */
    virtual void set_command(const size_t udriver_id, 
                             const MotorBoardCommand& command);

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

    /**
     * @brief Wait until the robot is ready.
     */
    void wait_until_ready();

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
        static_cast<SpiBus*>(instance_pointer)->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief This is the real time thread that streams the data to/from the
     * main board. 
     */
    void loop();

    /**
     * @brief Send the newest control stored in the time series. Some of the
     * command are not implemented/will be implemented. A warning is issued
     * upon miss-use.
     */
    void send_newest_command();

    /**
     * @brief Send the newest control stored in the time series.
     */
    void send_newest_controls();

    /**
     * Communication related attributes
     */

    /**
     * @brief Main board interface sdk:
     * https://github.com/open-dynamic-robot-initiative/master-board
     */
    std::shared_ptr<MasterBoardInterface> main_board_interface_;

    /**
     * @brief nb_udrivers_ is the number of the udriver controlled by the
     * main board.
     */
    size_t nb_udrivers_;

    /**
     * Outputs
     */

    /**
     * @brief All the measurements acquiered from the CAN
     * board.
     */
    std::vector<std::shared_ptr<MotorInterface::ScalarTimeseries> > measurement_;

    /**
     * @brief This is the status history of the udriver board.
     */
    std::vector<std::shared_ptr<MotorBoardInterface::StatusTimeseries> > status_;

    /**
     * Inputs
     */

    /**
     * @brief This is the buffer of the controls to be sent to card.
     */
    std::vector<std::shared_ptr<MotorInterface::ScalarTimeseries> > control_;

    /**
     * @brief This is the buffer of the commands to be sent to the card.
     */
    std::vector<std::shared_ptr<MotorBoardInterface::CommandTimeseries> >command_;

    /*! @brief history_length_ is the length of data buffers in number of
        iteration. */
    size_t history_length_;

    /**
     * Log
     */

    /**
     * @brief This is the history of the already sent controls.
     */
    std::vector<std::shared_ptr<MotorInterface::ScalarTimeseries> > sent_control_;

    /**
     * @brief This is the history of the already sent commands.
     */
    std::vector<std::shared_ptr<MotorBoardInterface::CommandTimeseries> > sent_command_;

    /**
     * Loop management
     */

    /**
     * @brief This boolean makes sure that the loop is stopped upon destruction
     * of this object.
     */
    bool is_loop_active_;

    /**
     * @brief Are motor in idle mode = 0 torques.
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

    /** @brief Everytime a motor index is detected the corresponding bit change
     * for the opposite value.
     * 
     * ```
     * if (index detected on motor X)
     * {
     *     motor_index_toggle_bits_[X] = !motor_index_toggle_bits_[X]
     * }
     * ```
     */
    std::array<bool, 12> motor_index_toggle_bits_;
};

}
