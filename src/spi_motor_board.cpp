/**
 * @file spi_motor_board.cpp
 * @author Felix Widmaier (felix.widmaier@tuebingen.mpg.de)
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file implements the classes from
 * "blmc_drivers/devices/motor_board.hpp"
 * @version 0.1
 * @date 2018-11-26
 *
 * @copyright Copyright (c) 2020, New York University and Max Planck Gesellschaft.
 *
 */

#include "real_time_tools/timer.hpp"
#include "blmc_drivers/devices/spi_motor_board.hpp"


namespace rt = real_time_tools;


namespace blmc_drivers
{

SpiMotorBoard::SpiMotorBoard(std::shared_ptr<SpiBus> spi_bus,
                             const size_t udriver_id): MotorBoardInterface()
{
  // Save the ser input
  spi_bus_ = spi_bus;
  udriver_id_ = udriver_id;
}

SpiMotorBoard::~SpiMotorBoard()
{
}

/**
 * Output and status
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries> SpiMotorBoard::
get_measurement(const int& index) const
{
    return spi_bus_->get_measurement(
        udriver_id_, (MotorBoardInterface::MeasurementIndex)index);
}

std::shared_ptr<const MotorBoardInterface::StatusTimeseries>
SpiMotorBoard::get_status() const
{
    return spi_bus_->get_status(udriver_id_);
}

/**
 * input logs
 */

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries>
SpiMotorBoard::get_control(const int& index) const
{
    return spi_bus_->get_control(
        udriver_id_, (MotorBoardInterface::ControlIndex)index);
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
SpiMotorBoard::get_command() const
{
    return spi_bus_->get_command(udriver_id_);
}

std::shared_ptr<const MotorBoardInterface::ScalarTimeseries> SpiMotorBoard::
get_sent_control(const int& index) const
{
    return spi_bus_->get_sent_control(
        udriver_id_, (MotorBoardInterface::ControlIndex)index);
}

std::shared_ptr<const MotorBoardInterface::CommandTimeseries>
SpiMotorBoard::get_sent_command() const
{
    return spi_bus_->get_sent_command(udriver_id_);
}

/**
 * Setters
 */

void SpiMotorBoard::set_control(const double& control,
                                const int& index)
{
    spi_bus_->set_control(
        udriver_id_, control, (MotorBoardInterface::ControlIndex)index);
}
    
void SpiMotorBoard::set_command(const MotorBoardCommand& command)
{
    spi_bus_->set_command(udriver_id_, command);
}

void SpiMotorBoard::send_if_input_changed()
{
    spi_bus_->send_if_input_changed();
}

} // namespace blmc_drivers
