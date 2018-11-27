/**
 * @file device_interface.hpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-27
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#pragma once

/**
 * @brief This namespace is the standard namespace of the package.
 */
namespace blmc_drivers
{

/**
 * @brief this class exists purely for logical reasons, it does not in
 * itself implement anything.
 *
 * the purpose of this class is to provide guidelines how a device should
 * be implemented. any device has a number of inputs and outputs, see
 * the following diagram for an example with two inputs and outputs.
 * @image html device_class_diagram.svg
 * generally, we expect the following functions to be implemented:
 * - a set function for each input (several inputs may share a set function
 * which takes an index argument).
 * - a send_if_input_changed() function which will send the inputs to the
 * device if any of them have changed.
 * - functions to access the current inputs and outputs, as well as the
 * inputs which have been sent to the device. rather than just returning
 * the latest elements, these function should return a ThreadsafeTimeseries
 * of these objects, such that the user can synchronize (e.g. wait for
 * the next element or step through them one by one such that none of them is
 * missed)
 */
class DeviceInterface
{

};

}
