
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <utils/threadsafe_timeseries.hpp>
#include <utils/timer.hpp>


typedef Eigen::Matrix<double, 20, 20> Type;

size_t length = 10000;
size_t n_outputs = 10;

std::vector<Type> inputs(length);
std::vector<std::vector<Type>> outputs(n_outputs, std::vector<Type>(length));

ThreadsafeTimeseries<Type> timeseries(length);



void
#ifndef __XENO__
    *
#endif
timeseries_to_output(void* void_ptr)
{
    size_t output_index = *static_cast<size_t*>(void_ptr);

    Timer<100> logger("timeseries_to_output " + std::to_string(output_index));

    for(size_t i = 0; i < length; i++)
    {
        Type element;
        ThreadsafeTimeseries<Type>::Index timeindex = i;
        element = timeseries[timeindex];
        outputs[output_index][i] = element;
        logger.end_and_start_interval();
    }

    logger.print_status();
}

void
#ifndef __XENO__
    *
#endif
input_to_timeseries(void* void_ptr)
{
    Timer<100> logger("input_to_timeseries");

    for(size_t i = 0; i < length; i++)
    {
        timeseries.append(inputs[i]);
        logger.end_and_start_interval();
    }

    logger.print_status();
}


TEST(threadsafe_timeseries, full_history)
{
    timeseries = ThreadsafeTimeseries<Type>(length);

    srand(0);
    for(size_t i = 0; i < inputs.size(); i++)
    {
        inputs[i] = Type::Random();
    }

    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_print_auto_init(1);

    std::vector<size_t> output_indices(n_outputs);
    for(size_t i = 0; i < n_outputs; i++)
    {
        output_indices[i] = i;
        osi::start_thread(timeseries_to_output, &output_indices[i]);
    }
    usleep(1000);

    osi::start_thread(input_to_timeseries);
    usleep(1000000);

    // check that the outputs written by the individual threads
    // correspond to the input.
    for(size_t i = 0; i < n_outputs; i++)
    {
        EXPECT_TRUE(inputs == outputs[i]);
    }

    // sanity check
    inputs[0](0,0) = 33.;
    EXPECT_FALSE(inputs == outputs[0]);
}



void
#ifndef __XENO__
    *
#endif
input_to_timeseries_slow(void* void_ptr)
{
    Timer<100> logger("input_to_timeseries");

    for(size_t i = 0; i < length; i++)
    {
        timeseries.append(inputs[i]);
        logger.end_and_start_interval();

        usleep(1);
    }

    logger.print_status();
}


TEST(threadsafe_timeseries, partial_history)
{
    timeseries = ThreadsafeTimeseries<Type>(100);


    srand(0);
    for(size_t i = 0; i < inputs.size(); i++)
    {
        inputs[i] = Type::Random();
    }

    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_print_auto_init(1);

    std::vector<size_t> output_indices(n_outputs);
    for(size_t i = 0; i < n_outputs; i++)
    {
        output_indices[i] = i;
        osi::start_thread(timeseries_to_output, &output_indices[i]);
    }
    usleep(1000);

    osi::start_thread(input_to_timeseries_slow);
    usleep(1000000);

    // check that the outputs written by the individual threads
    // correspond to the input.
    for(size_t i = 0; i < n_outputs; i++)
    {
        EXPECT_TRUE(inputs == outputs[i]);
    }

    // sanity check
    inputs[0](0,0) = 33.;
    EXPECT_FALSE(inputs == outputs[0]);
}

