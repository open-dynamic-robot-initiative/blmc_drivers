/**
 * @file can_bus.hpp
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-07-11
 */

#pragma once

#include <memory>
#include <string>

#include <real_time_tools/iostream.hpp>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_object.hpp>
#include <real_time_tools/timer.hpp>

#include <time_series/time_series.hpp>

#include "blmc_drivers/devices/device_interface.hpp"
#include "blmc_drivers/utils/os_interface.hpp"

namespace blmc_drivers
{
/**
 * @brief CanBusFrame is a class that contains a fixed sized amount of data
 * to be send or received via the can bus
 */
class CanBusFrame
{
public:
    /**
     * @brief data is the acutal data to be sent/received.
     */
    std::array<uint8_t, 8> data;
    /**
     * @brief  dlc is the size of the message.
     */
    uint8_t dlc;
    /**
     * @brief id is the id number return by the CAN bus.
     */
    can_id_t id;

    void print() const
    {
        rt_printf("---------------------------\n");
        rt_printf("can bus frame data");
        for (auto& datum : data)
        {
            rt_printf(" :%d", datum);
        }
        rt_printf("\n");

        rt_printf("id: %d\n", id);

        rt_printf("dlc: %d\n", dlc);

        rt_printf("---------------------------\n");
    }
};

/**
 * @brief CanBusConnection is a data structure that contains the hardware
 * details for the connection between to can cards.
 */
class CanBusConnection
{
public:
    /**
     * @brief send_addr is the ip address where to send the the messages.
     */
    struct sockaddr_can send_addr;
    /**
     * @brief socket is the port through which the messages will be processed
     */
    int socket;
};

/**
 * @brief CanBusInterface is an abstract class that defines an API for the
 * communication via Can bus.
 */
class CanBusInterface : public DeviceInterface
{
public:
    /**
     * @brief Destroy the CanBusInterface object
     */
    virtual ~CanBusInterface()
    {
    }

    /**
     * @brief CanframeTimeseries is a simple sohortcut
     */
    typedef time_series::TimeSeries<CanBusFrame> CanframeTimeseries;

    /**
     * getters
     */

    /**
     * @brief Get the output frame
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    virtual std::shared_ptr<const CanframeTimeseries> get_output_frame()
        const = 0;

    /**
     * @brief Get the input frame
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    virtual std::shared_ptr<const CanframeTimeseries> get_input_frame() = 0;

    /**
     * @brief Get the sent input frame
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    virtual std::shared_ptr<const CanframeTimeseries>
    get_sent_input_frame() = 0;

    /**
     * setters
     */

    /**
     * @brief Set the input frame saves the input frame to be sent in a queue.
     *
     * @param input_frame
     */
    virtual void set_input_frame(const CanBusFrame& input_frame) = 0;

    /**
     * Sender
     */

    /**
     * @brief send all the input frame to the can network
     */
    virtual void send_if_input_changed() = 0;
};

/**
 * @brief CanBus is the implementation of the CanBusInterface.
 */
class CanBus : public CanBusInterface
{
public:
    /**
     * @brief Construct a new CanBus object
     *
     * @param can_interface_name
     * @param history_length
     */
    CanBus(const std::string& can_interface_name,
           const size_t& history_length = 1000);

    /**
     * @brief Destroy the CanBus object
     */
    virtual ~CanBus();

    /**
     * Getters
     */

    /**
     * @brief Get the output frame
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    std::shared_ptr<const CanframeTimeseries> get_output_frame() const
    {
        return output_;
    }

    /**
     * @brief Get the input frame
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    virtual std::shared_ptr<const CanframeTimeseries> get_input_frame()
    {
        return input_;
    }

    /**
     * @brief Get the input frame thas has been sent
     *
     * @return std::shared_ptr<const CanframeTimeseries>
     */
    virtual std::shared_ptr<const CanframeTimeseries> get_sent_input_frame()
    {
        return sent_input_;
    }

    /**
     * @brief Setters
     */

    /**
     * @brief Set the input frame
     *
     * @param input_frame
     */
    virtual void set_input_frame(const CanBusFrame& input_frame)
    {
        input_->append(input_frame);
    }

    /**
     * @brief Sender
     */

    /**
     * @brief Send the queue of message to the can network
     */
    virtual void send_if_input_changed();

    /**
     * private attributes and methods
     */
private:
    /**
     * @brief This function is an helper that allows us to launch real-time
     * thread in xenaomai, ubunt, or rt-preempt seemlessly.
     *
     * @param instance_pointer
     * @return THREAD_FUNCTION_RETURN_TYPE (is void or void* depending on the
     * OS.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer)
    {
        ((CanBus*)(instance_pointer))->loop();
        return THREAD_FUNCTION_RETURN_VALUE;
    }

    /**
     * @brief Execute the communication loop with the can bus
     */
    void loop();

    /**
     * @brief Send input data
     *
     * @param unstamped_can_frame is a frame without id nor time.
     */
    void send_frame(const CanBusFrame& unstamped_can_frame);

    /**
     * @brief Get the output frame from the bus
     *
     * @return CanBusFrame is the output frame data.
     */
    CanBusFrame receive_frame();

    /**
     * @brief Setup and initialize the CanBus object.
     * It connects to the can bus. This method is used once in the constructor.
     *
     * @param name is the can card name.
     * @param err_mask, always used with "0" so far (TODO: Manuel explain)
     * @return CanBusConnection
     */
    CanBusConnection setup_can(std::string name, uint32_t err_mask);

    /**
     * Attributes
     */

    /**
     * @brief can_connection_ is the communication object allowing to send or
     * receive can frames.
     */
    real_time_tools::SingletypeThreadsafeObject<CanBusConnection, 1>
        can_connection_;

    /**
     * @brief input_ is a list of time stamped frame to be send to the can
     * network.
     */
    std::shared_ptr<time_series::TimeSeries<CanBusFrame> > input_;

    /**
     * @brief sent_inupt_ is the list of the input already sent to the network.
     */
    std::shared_ptr<time_series::TimeSeries<CanBusFrame> > sent_input_;

    /**
     * @brief output_ is the list of the frames received from the can network.
     */
    std::shared_ptr<time_series::TimeSeries<CanBusFrame> > output_;

    /**
     * @brief This boolean makes sure that the loop is not active upon
     * destruction of the current object
     */
    bool is_loop_active_;

    /**
     * @brief rt_thread_ is the thread object allowing us to spawn real-time
     * threads.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief Log directory.
     */
    std::string log_dir_;

    /**
     * @brief time_log_name is the name of the loggin
     */
    std::string name_;
};

}  // namespace blmc_drivers
