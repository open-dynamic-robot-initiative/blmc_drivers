/**
 * @file serial_reader.cpp
 * @author Julian Viereck
 * \brief Wrapper for reading new-line terminated list of values from serial
 * port.
 * @date 2020-01-24
 *
 * @copyright Copyright (c) 2018
 *
 */

#include "blmc_drivers/serial_reader.hpp"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>

#include <unistd.h>
#include <fstream>
#include <iostream>

namespace rt = real_time_tools;

namespace blmc_drivers
{
SerialReader::SerialReader(const std::string &serial_port,
                           const int &num_values)
{
    std::string serial_port_try;
    for (int i = 0; i < 4; ++i)
    {
        // HACK: Ignore the provided serial port and
        if (i == 0)
        {
            serial_port_try = "/dev/ttyACM";
        }
        else if (i == 1)
        {
            serial_port_try = "/dev/ttyACM0";
        }
        else if (i == 2)
        {
            serial_port_try = "/dev/ttyACM1";
        }
        else if (i == 3)
        {
            serial_port_try = serial_port;
        }
        else
        {
            std::cerr << "Unable to open serial port";
            has_error_ = true;
            return;
        }
        std::cout << "Try to open serial port at " << serial_port_try
                  << std::endl;
        fd_ = open(serial_port_try.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ != -1)
        {
            std::cout << "Opened serial port at " << serial_port_try
                      << std::endl;
            break;
        }
    }

    struct termios options;

    fcntl(fd_, F_SETFL, FNDELAY);  // Open the device in nonblocking mode

    // Set parameters
    tcgetattr(fd_, &options);          // Get the current options of the port
    bzero(&options, sizeof(options));  // Clear all the options
    speed_t Speed = B115200;
    cfsetispeed(&options, Speed);  // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);
    options.c_cflag |=
        (CLOCAL | CREAD |
         CS8);  // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= (IGNPAR | IGNBRK);
    options.c_cc[VTIME] = 0;  // Timer unused
    options.c_cc[VMIN] = 0;   // At least on character before satisfy reading
    tcsetattr(fd_, TCSANOW, &options);  // Activate the settings

    latest_values_.resize(num_values);

    has_error_ = false;
    is_active_ = false;
    is_loop_active_ = true;
    // Launch the main processing loop.
    rt_thread_.create_realtime_thread(&SerialReader::loop, this);
}

void SerialReader::loop()
{
    int byte_read;
    int buffer_size = 128;
    char buffer[buffer_size];
    char line[buffer_size];
    int line_index = 0;

    is_active_ = true;
    while (is_loop_active_)
    {
        int byte_consumed = 0;
        byte_read = read(fd_, buffer, buffer_size);
        while (byte_consumed < byte_read)
        {
            line[line_index++] = buffer[byte_consumed++];
            if (buffer[byte_consumed - 1] == '\n')
            {
                // Ignore the "\r\n" in the string.
                line[line_index - 1] = '\0';
                line[line_index - 2] = '\0';
                line_index -= 2;

                // Read the actual numbers from the line.
                int bytes_scanned_total = 0;
                int bytes_scanned;
                int number;
                mutex_.lock();
                for (std::size_t i = 0; i < latest_values_.size(); i++)
                {
                    sscanf(line + bytes_scanned_total,
                           "%d %n",
                           &number,
                           &bytes_scanned);
                    if (bytes_scanned_total >= line_index)
                    {
                        break;
                    }
                    bytes_scanned_total += bytes_scanned;
                    latest_values_[i] = number;
                }
                new_data_counter_ += 1;
                mutex_.unlock();

                line_index = 0;
            }
        }
        usleep(100);
    }
}

SerialReader::~SerialReader()
{
    is_loop_active_ = false;
    rt_thread_.join();
    close(fd_);
}

int SerialReader::fill_vector(std::vector<int> &values)
{
    mutex_.lock();
    if (new_data_counter_ == 0)
    {
        missed_data_counter_ += 1;
    }
    else
    {
        missed_data_counter_ = 0;
    }
    new_data_counter_ = 0;
    values = latest_values_;
    mutex_.unlock();

    return missed_data_counter_;
}

}  // namespace blmc_drivers
