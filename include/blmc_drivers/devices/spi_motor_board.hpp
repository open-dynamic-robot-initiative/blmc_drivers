/**
 * @file spi_motor_board.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Interface for the master board designed by Thomas Floyols
 * https://github.com/open-dynamic-robot-initiative/master-board
 * @version 0.1
 * @date 2019-11-14
 * 
 * @copyright Copyright (c) 2019-2020, New York University and Max Planck Gesellshaft.
 * 
 */

#pragma once

#include "blmc_drivers/devices/spi_bus.hpp"
#include "real_time_tools/thread.hpp"
#include "real_time_tools/threadsafe/threadsafe_timeseries.hpp"

#include "blmc_drivers/devices/motor_board.hpp"

namespace blmc_drivers
{

class SpiMotorBoard: public MotorBoardInterface
{

public:
    /**
     * @brief Construct a new SpiMotorBoard object
     * 
     * The constructor starts a real time thread: SpiMotorBoard::loop().
     * This thread streams the data back and forth collecting the sensor data
     * and sends the control/commands.
     * 
     * @param spi_bus is the object that communicate with the
     * master board which in turn communicate with the motor boards.
     * The master board provides the hardware informations and sends the
     * commands.
     * @param udriver_id is the id of the udriver this class represents.
     */
    SpiMotorBoard(std::shared_ptr<SpiBus> spi_bus, const size_t udriver_id);

    /**
     * @brief Destroy the SpiMotorBoard object. The destructor handles
     * the proper shutdown of the class and the threads.
     */
    ~SpiMotorBoard();

    /**
     * Output and status
     */

    /**
     * @copydoc MotorBoardInterface::get_measurement()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries>
    get_measurement(const int& index) const ;

    /**
     * @copydoc MotorBoardInterface::get_status()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::StatusTimeseries>
    get_status() const;

    /**
     * input logs
     */

    /**
     * @copydoc MotorBoardInterface::get_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries>
    get_control(const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
    get_command() const;

    /**
     * @copydoc MotorBoardInterface::get_sent_control()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorInterface::ScalarTimeseries>
    get_sent_control(const int& index) const;

    /**
     * @copydoc MotorBoardInterface::get_sent_command()
     * Inherited from MotorBoardInterface
     */
    virtual std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
    get_sent_command() const;

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
     * Communication related attributes
     */

    /**
     * @brief Master board interface sdk:
     * https://github.com/open-dynamic-robot-initiative/master-board
     */
    std::shared_ptr<SpiBus> spi_bus_;

    /**
     * @brief udriver_id_ is the index of the udriver controlled by the
     * master board. The index is the one used in MasterBoardInterface. The
     * index should correspond to the hardware SPI index onboard the card.
     */
    size_t udriver_id_;
};

}