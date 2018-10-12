#include <eigen3/Eigen/Core>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sched.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include <devices/analog_sensor.hpp>

#include <utils/timer.hpp>



double protected_value;
std::shared_ptr<osi::Mutex> mutex;


static THREAD_FUNCTION_RETURN_TYPE thread_body_locking(void* index)
{
    int print_counter = 10000000;

    //    cpu_set_t cpuset;
    //    CPU_ZERO(&cpuset);
    //    CPU_SET(*((int*)(index)) + 1, &cpuset);
    //    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    // if (rc != 0) {
    // printf("Error calling pthread_setaffinity_np: %d\n", rc);
    // }

    osi::realtime_printf("Hello world from thread id=%d, cpu=%d\n", syscall(SYS_gettid), sched_getcpu());

    while(true)
    {
        Timer<1> timer;

        double element = 0.;
        size_t cpu_index = sched_getcpu();
        bool cpu_switch = false;
        for(size_t i = 0; i < print_counter; i++)
        {
            if(sched_getcpu() != cpu_index)
            {
                cpu_switch = true;
            }

            timer.end_and_start_interval();

//            osi::sleep_ms(0.000000001);
            {
                std::unique_lock<osi::Mutex> lock(*mutex);
            }

            element += 1.;
        }
        osi::realtime_printf("%d store and reads from thread %d, cpu=%d, cpu_switch=%d\n",
                             print_counter,
                             *(int*)(index),
                             sched_getcpu(),
                             cpu_switch);
        timer.print_status();
    }
}



int main(int argc, char **argv)
{
    mutex = std::make_shared<osi::Mutex>();

    mlockall(MCL_CURRENT | MCL_FUTURE);

    osi::initialize_realtime_printing();

    osi::realtime_printf("argc= %d\n", argc);

    std::vector<int> indices(4);
    for(size_t i = 0; i < indices.size(); i++)
        indices[i] = i;

    for (int i = 0; i < 4; i++)
    {
        osi::start_thread(&thread_body_locking, &indices[i]);
    }


    while(true)
    {
        Timer<>::sleep_ms(10);
    }
}
