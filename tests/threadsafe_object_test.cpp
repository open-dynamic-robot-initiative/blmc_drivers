
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <blmc_drivers/utils/threadsafe_object.hpp>
#include <real_time_tools/timer.hpp>

#include <tuple>

using namespace blmc_drivers;
using namespace real_time_tools;


const int DATA_LENGTH = 10000;
const int OUTPUT_COUNT = 5;

typedef int Type0;
typedef Eigen::Matrix3d Type1;
typedef double Type2;
typedef Eigen::Matrix<double, 20, 20> Type3;


typedef std::tuple<
std::array<Type0, DATA_LENGTH>,
std::array<Type1, DATA_LENGTH>,
std::array<Type2, DATA_LENGTH>,
std::array<Type3, DATA_LENGTH>
> DataType;

DataType input_data;
std::array< DataType, OUTPUT_COUNT> output_data;



template <int INDEX, typename ThreadsafeObjectType>
void
#ifndef __XENO__
    *
#endif
input_function(void* void_ptr)
{
    ThreadsafeObjectType* threadsafe_object_ptr =
            static_cast<ThreadsafeObjectType*>(void_ptr);

    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("input " + std::to_string(INDEX));
    
    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        threadsafe_object_ptr->template set<INDEX>(
                    std::get<INDEX>(input_data)[i]);
        logger.tac_tic();
    }

    logger.print_statistics();
}




template <int DATA_INDEX, int OUTPUT_INDEX, typename ThreadsafeObjectType>
void
#ifndef __XENO__
    *
#endif
output_function(void * void_ptr)
{
    ThreadsafeObjectType* threadsafe_object_ptr =
            static_cast<ThreadsafeObjectType*>(void_ptr);

    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("output " + std::to_string(DATA_INDEX) +
                    ", " + std::to_string(OUTPUT_INDEX));

    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        threadsafe_object_ptr->template wait_for_update<DATA_INDEX>();
        std::get<DATA_INDEX>(output_data[OUTPUT_INDEX])[i] =
                threadsafe_object_ptr->template get<DATA_INDEX>();
        logger.tac_tic();
    }

    //    logger.print_statistics();
}


template <int OUTPUT_INDEX, typename ThreadsafeObjectType>
void
#ifndef __XENO__
    *
#endif
complete_output_function(void * void_ptr)
{
    ThreadsafeObjectType* threadsafe_object_ptr =
            static_cast<ThreadsafeObjectType*>(void_ptr);

    Timer logger;
    logger.set_memory_size(100);
    logger.set_name("complete output " + std::to_string(OUTPUT_INDEX));

    int i_0, i_1, i_2, i_3 = 0;
    for(size_t i = 0; i < 4 * DATA_LENGTH; i++)
    {
        unsigned data_index = threadsafe_object_ptr->wait_for_update();
        switch(data_index)
        {
        case 0:
            std::get<0>(output_data[OUTPUT_INDEX])[i_0++] =
                    threadsafe_object_ptr-> template get<0>();
            break;
        case 1:
            std::get<1>(output_data[OUTPUT_INDEX])[i_1++] =
                    threadsafe_object_ptr-> template get<1>();
            break;
        case 2:
            std::get<2>(output_data[OUTPUT_INDEX])[i_2++] =
                    threadsafe_object_ptr-> template get<2>();
            break;
        case 3:
            std::get<3>(output_data[OUTPUT_INDEX])[i_3++] =
                    threadsafe_object_ptr-> template get<3>();
            break;
        }
        logger.tac_tic();
    }

    logger.print_statistics();
}


void print(double value)
{
    std::cout << value << std::endl;
}

void initialize_data_randomly()
{
    // initialize inputs -------------------------------------------------------
    srand(0);
    for(size_t i = 0; i < DATA_LENGTH; i++)
    {
        std::get<0>(input_data)[i] = rand();
        std::get<1>(input_data)[i] = Type1::Random();
        std::get<2>(input_data)[i] = rand() / 1223232.0;
        std::get<3>(input_data)[i] = Type3::Random();
    }

}


//TEST(threadsafe_object, single_input_multi_output)
//{
//    typedef ThreadsafeObject<Type0, Type1, Type2, Type3> TestType1;
//    TestType1 test_object_1;

//    initialize_data_randomly();



//    mlockall(MCL_CURRENT | MCL_FUTURE);
//    osi::initialize_realtime_printing();

//    // start a thread for each output and input function -----------------------
//    osi::start_thread(output_function<0,0,TestType1>, &test_object_1);
//    osi::start_thread(output_function<0,1,TestType1>, &test_object_1);
//    osi::start_thread(output_function<0,2,TestType1>, &test_object_1);
//    osi::start_thread(output_function<0,3,TestType1>, &test_object_1);

//    osi::start_thread(output_function<1,0,TestType1>, &test_object_1);
//    osi::start_thread(output_function<1,1,TestType1>, &test_object_1);
//    osi::start_thread(output_function<1,2,TestType1>, &test_object_1);
//    osi::start_thread(output_function<1,3,TestType1>, &test_object_1);

//    osi::start_thread(output_function<2,0,TestType1>, &test_object_1);
//    osi::start_thread(output_function<2,1,TestType1>, &test_object_1);
//    osi::start_thread(output_function<2,2,TestType1>, &test_object_1);
//    osi::start_thread(output_function<2,3,TestType1>, &test_object_1);

//    osi::start_thread(output_function<3,0,TestType1>, &test_object_1);
//    osi::start_thread(output_function<3,1,TestType1>, &test_object_1);
//    osi::start_thread(output_function<3,2,TestType1>, &test_object_1);
//    osi::start_thread(output_function<3,3,TestType1>, &test_object_1);

//    osi::start_thread(complete_output_function<4, TestType1>, &test_object_1);

//    usleep(1000);

//    auto task_1 = osi::start_thread(input_function<0, TestType1>, &test_object_1);
//    auto task_2 = osi::start_thread(input_function<1, TestType1>, &test_object_1);
//    auto task_3 = osi::start_thread(input_function<2, TestType1>, &test_object_1);
//    auto task_4 = osi::start_thread(input_function<3, TestType1>, &test_object_1);

//    usleep(1000000);

//    // check that the outputs written by the individual threads
//    // correspond to the input.
//    EXPECT_TRUE(input_data == output_data[0]);
//    EXPECT_TRUE(input_data == output_data[1]);
//    EXPECT_TRUE(input_data == output_data[2]);
//    EXPECT_TRUE(input_data == output_data[3]);
//    EXPECT_TRUE(input_data == output_data[4]);



//    // sanity check
//    std::get<1>(input_data)[0](1,1) = 33.;
//    EXPECT_FALSE(input_data == output_data[0]);
//}



