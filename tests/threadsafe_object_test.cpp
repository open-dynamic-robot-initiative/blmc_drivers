
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <threadsafe_object.hpp>
#include <time_logger.hpp>

#include <tuple>




const int DATA_LENGTH = 10000;

const int OUTPUT_COUNT = 4;

const double RATE_MS = 0.01;


typedef int Type0;
typedef Eigen::Matrix3d Type1;
typedef double Type2;
typedef Eigen::Matrix<double, 20, 20> Type3;


typedef std::tuple<
std::array<Type0, DATA_LENGTH>,
std::array<Type1, DATA_LENGTH>,
std::array<Type2, DATA_LENGTH>,
std::array<Type3, DATA_LENGTH>
> DataTuple;

DataTuple input_tuple;
std::array< DataTuple, OUTPUT_COUNT> output_tuples;

ThreadsafeObject<Type0, Type1, Type2, Type3> threadsafe_object;

template <int INDEX> void input_function(void * bla)
{
    TimeLogger<100> logger("input " + std::to_string(INDEX));

    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        rt_task_sleep(int(RATE_MS * 1000000.));
        threadsafe_object.set<INDEX>(std::get<INDEX>(input_tuple)[i]);
        logger.end_and_start_interval();
    }

    logger.print_status();
}


template <int DATA_INDEX, int OUTPUT_INDEX> void output_function(void * trash)
{

    TimeLogger<100> logger("output " + std::to_string(DATA_INDEX) + ", " + std::to_string(OUTPUT_INDEX));


    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        threadsafe_object.wait_for_datum(DATA_INDEX);
        std::get<DATA_INDEX>(output_tuples[OUTPUT_INDEX])[i] = threadsafe_object.get<DATA_INDEX>();
        logger.end_and_start_interval();
    }

//    logger.print_status();
}


void start_thread(void (*function)(void *cookie))
{

    RT_TASK trash;
    int priority = 10;

    int return_task_create = rt_task_create(&trash, NULL, 0, priority,  T_JOINABLE | T_FPU);
    if (return_task_create) {
        rt_fprintf(stderr, "controller: %s\n", strerror(-return_task_create));
        exit(-1);
    }
    rt_task_start(&trash, function, NULL);
}


TEST(threadsafe_object, general)
{
    // initialize inputs ------------------------------------------
    srand(0);
    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        std::get<0>(input_tuple)[i] = rand();
        std::get<1>(input_tuple)[i] = Type1::Random();
        std::get<2>(input_tuple)[i] = rand() / 1223232.0;
        std::get<3>(input_tuple)[i] = Type3::Random();
    }

    // start a thread for each output and input function ----------
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_print_auto_init(1);

    start_thread(output_function<0,0>);
    start_thread(output_function<0,1>);
    start_thread(output_function<0,2>);
    start_thread(output_function<0,3>);

    start_thread(output_function<1,0>);
    start_thread(output_function<1,1>);
    start_thread(output_function<1,2>);
    start_thread(output_function<1,3>);

    start_thread(output_function<2,0>);
    start_thread(output_function<2,1>);
    start_thread(output_function<2,2>);
    start_thread(output_function<2,3>);

    start_thread(output_function<3,0>);
    start_thread(output_function<3,1>);
    start_thread(output_function<3,2>);
    start_thread(output_function<3,3>);


    usleep(1000);

    start_thread(input_function<0>);
    start_thread(input_function<1>);
    start_thread(input_function<2>);
    start_thread(input_function<3>);



    usleep(DATA_LENGTH * RATE_MS * 1000 * 2);
    usleep(1000000);


    // check that the outputs written by the individual threads
    // correspond to the input.
    EXPECT_TRUE(input_tuple == output_tuples[0]);
    EXPECT_TRUE(input_tuple == output_tuples[1]);
    EXPECT_TRUE(input_tuple == output_tuples[2]);
    EXPECT_TRUE(input_tuple == output_tuples[3]);

    // sanity check
    std::get<1>(input_tuple)[0](1,1) = 33.;
    EXPECT_FALSE(input_tuple == output_tuples[0]);
}



