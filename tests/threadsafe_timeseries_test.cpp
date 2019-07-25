
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <blmc_drivers/utils/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>
#include "real_time_tools/thread.hpp"

using namespace blmc_drivers;
using namespace real_time_tools;

typedef Eigen::Matrix<double, 20, 20> Type;

size_t length = 10000;
size_t n_outputs = 10;

std::vector<Type> inputs(length);
std::vector<std::vector<Type>> outputs(n_outputs, std::vector<Type>(length));

ThreadsafeTimeseries<Type> timeseries(length);



void * timeseries_to_output(void* void_ptr)
{
    size_t output_index = *static_cast<size_t*>(void_ptr);

    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("timeseries_to_output " + std::to_string(output_index));

    for(size_t i = 0; i < length; i++)
    {
        Type element;
        ThreadsafeTimeseries<Type>::Index timeindex = i;
        element = timeseries[timeindex];
        outputs[output_index][i] = element;
        logger.tac_tic();
    }

//    logger.print_statistics();
    return nullptr;
}

void * input_to_timeseries(void* void_ptr)
{
    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("input_to_timeseries");

    for(size_t i = 0; i < length; i++)
    {
        timeseries.append(inputs[i]);
        logger.tac_tic();
    }

//    logger.print_statistics();
    return nullptr;
}


TEST(threadsafe_timeseries, full_history)
{
    timeseries = ThreadsafeTimeseries<Type>(length);

    srand(0);
    for(size_t i = 0; i < inputs.size(); i++)
    {
        inputs[i] = Type::Random();
    }

    std::vector<size_t> output_indices(n_outputs);

    std::vector<RealTimeThread> threads(n_outputs);
    for(size_t i = 0; i < n_outputs; i++)
    {
        output_indices[i] = i;
        threads[i].create_realtime_thread(
          &timeseries_to_output, &output_indices[i]);
    }
    usleep(1000);

    RealTimeThread input_to_timeseries_thread;
    input_to_timeseries_thread.create_realtime_thread(&input_to_timeseries);
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



void * input_to_timeseries_slow(void* void_ptr)
{
    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("input_to_timeseries");

    for(size_t i = 0; i < length; i++)
    {
        timeseries.append(inputs[i]);
        logger.tac_tic();

        usleep(100);
    }

//    logger.print_statistics();

    return nullptr;
}


TEST(threadsafe_timeseries, partial_history)
{
    timeseries = ThreadsafeTimeseries<Type>(100);


    srand(0);
    for(size_t i = 0; i < inputs.size(); i++)
    {
        inputs[i] = Type::Random();
    }

    std::vector<RealTimeThread> threads(n_outputs + 1);
    std::vector<size_t> output_indices(n_outputs);
    for(size_t i = 0; i < n_outputs; i++)
    {
        output_indices[i] = i;
        threads[i].create_realtime_thread(
          &timeseries_to_output, &output_indices[i]);
    }
    usleep(1000);
    threads[n_outputs].create_realtime_thread(
      &input_to_timeseries_slow);
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

