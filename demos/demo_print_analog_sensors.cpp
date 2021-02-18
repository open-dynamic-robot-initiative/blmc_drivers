/**
 * @file
 * @brief Print ADC measurements in a loop.
 * @copyright 2018-2020, New York University and Max Planck Gesellschaft,
 *            License BSD-3-Clause
 */
#include <iomanip>

#include <real_time_tools/spinner.hpp>

#include <blmc_drivers/devices/analog_sensor.hpp>
#include <blmc_drivers/devices/can_bus.hpp>
#include <blmc_drivers/devices/motor_board.hpp>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Invalid number of arguments." << std::endl;
        std::cerr << "Usage:  " << argv[0] << " <can_port>" << std::endl;
        return 1;
    }

    std::string can_port(argv[1]);

    // First of all one need to initialize the communication with the can bus.
    auto can_bus = std::make_shared<blmc_drivers::CanBus>(can_port);

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: The BLMC motors are aligned during this stage.
    auto motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus);

    // create analogue sensors objects to get measurements
    auto adc_a = std::make_shared<blmc_drivers::AnalogSensor>(motor_board, 0);
    auto adc_b = std::make_shared<blmc_drivers::AnalogSensor>(motor_board, 1);

    std::cout << std::endl;
    std::cout << "Printing measurements of analogue sensors 'ADC A' and 'ADC "
                 "B'.  Press Ctrl+C to exit."
              << std::endl;
    std::cout << std::endl;

    // print measurements in a loop
    real_time_tools::Spinner spinner;
    spinner.set_period(0.05);
    while (true)
    {
        double measurement_a = adc_a->get_measurement()->newest_element();
        double measurement_b = adc_b->get_measurement()->newest_element();

        std::cout << std::setprecision(4) << std::fixed
                  << "\rADC A: " << measurement_a
                  << " | ADC B: " << measurement_b << "      " << std::flush;

        spinner.spin();
    }

    return 0;
}
