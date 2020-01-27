/**
 * @file serial_reader.cpp
 * @author Julian Viereck
 * \brief Wrapper for reading new-line terminated list of values from serial port.
 * @date 2020-01-24
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "blmc_drivers/serial_reader.hpp"
#include <stdexcept>



namespace rt = real_time_tools;

namespace blmc_drivers
{

SerialReader::SerialReader(const std::string &serial_port, const int &num_values)
{
    if (num_values != 4) {
        throw std::invalid_argument("Only reading of 4 values allowed.");
    }

    std::string serial_port_try;
    for (int i=0; i < 4; i++) {
        // HACK: Ignore the provided serial port and
        if (i == 0) {
            serial_port_try = "/dev/ttyACM";
        } else if (i == 1) {
            serial_port_try = "/dev/ttyACM0";
        } else if (i == 2) {
            serial_port_try = "/dev/ttyACM1";
        } else {
            std::cerr << "Unable to open serial port";
            has_error_ = true;
            return;
        }
        try {
            std::cout << "Try to open serial port at " << serial_port_try << std::endl;
            serial_stream_.Open(serial_port_try);
            std::cout << "Opened serial port at " << serial_port_try << std::endl;
            break;
        } catch (const OpenFailed&) {
            serial_stream_.Close();
        }
    }

    // std::cout << "Attempt to open serial port " << serial_port << std::endl;
    // try {
    //     serial_stream_.Open(serial_port);
    // } catch (const OpenFailed&) {
    //     has_error_ = true;
    //     std::cerr << "Unable to open serial port";
    //     return;
    // }

    // Set the baud rate of the serial port.
    serial_stream_.SetBaudRate(BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_stream_.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    serial_stream_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    serial_stream_.SetParity(Parity::PARITY_NONE) ;

    // Set the number of stop bits.
    serial_stream_.SetStopBits(StopBits::STOP_BITS_1) ;

    latest_values_.resize(num_values);

    has_error_ = false;
    is_active_ = false;
    is_loop_active_ = true;
    // Launch the main processing loop.
    rt_thread_.create_realtime_thread(&SerialReader::loop, this);
}

void SerialReader::loop()
{
    int i = 0;
    int j = 0;

    int a0, a1, a2, a3;

    const int buffer_size = 128;
    char buffer[buffer_size];

    // TODO: Convert this to `SerialPort.ReadLine()`.

    // Wait for data to be available at the serial port.
    while(serial_stream_.rdbuf()->in_avail() == 0) {
        usleep(1000);
    }

    is_active_ = true;
    while (is_loop_active_) {
        // Keep reading data from serial port and print it to the screen.
        while(serial_stream_.IsDataAvailable())
        {
            // Variable to store data coming from the serial port.
            char data_byte ;

            // Read a single byte of data from the serial port.
            serial_stream_.get(data_byte) ;

            if (data_byte == '\n') {

                buffer[i] = '\0';
                // std::cout << buffer << std::endl;
                if (sscanf(buffer, "%d %d %d %d", &a0, &a1, &a2, &a3) == 4) {
                    mutex_.lock();
                    latest_values_[0] = a0;
                    latest_values_[1] = a1;
                    latest_values_[2] = a2;
                    latest_values_[3] = a3;
                    new_data_counter_ += 1;
                    mutex_.unlock();
                } else {
                    std::cout << "Failed to read all four numbers." << std::endl;
                }

                i = 0;
            } else {
                buffer[i] = data_byte;
                i++;
            }
        }
        usleep(100) ;
    }
}

SerialReader::~SerialReader()
{
    is_loop_active_ = false;
    rt_thread_.join();
    serial_stream_.Close();
}

int SerialReader::fill_vector(std::vector<int>& values)
{
    mutex_.lock();
    if (new_data_counter_ == 0) {
        missed_data_counter_ += 1;
    } else {
        missed_data_counter_ = 0;
    }
    new_data_counter_ = 0;
    values = latest_values_;
    mutex_.unlock();

    return missed_data_counter_;
}

} // namespace blmc_drivers
