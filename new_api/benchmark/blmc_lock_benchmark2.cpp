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


ThreadsafeTimeseries<double> series(1000);


typedef long int Index;
typedef long double Timestamp;

std::shared_ptr<std::vector<double>> history_elements_;
std::shared_ptr<std::vector<Timestamp>> history_timestamps_;

Index oldest_timeindex_;
Index newest_timeindex_;
Index tagged_timeindex_;

std::shared_ptr<osi::ConditionVariable> condition_;
std::shared_ptr<osi::Mutex> mutex_;







static THREAD_FUNCTION_RETURN_TYPE thread_body_locking(void* index)
{
    int print_counter = 1000000;

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


            // append ----------------------------------------------------------
            {
                std::unique_lock<osi::Mutex> lock(*mutex_);
                newest_timeindex_++;
                if(newest_timeindex_ - oldest_timeindex_ + 1
                        > history_elements_->size())
                {
                    oldest_timeindex_++;
                }
                Index history_index = newest_timeindex_ % history_elements_->size();
                (*history_elements_)[history_index] = element;
                (*history_timestamps_)[history_index] = Timer<>::current_time_ms();
            }
            condition_->notify_all();
            // -----------------------------------------------------------------
            Index timeindex;
            {
                std::unique_lock<osi::Mutex> lock(*mutex_);
                while(newest_timeindex_ < oldest_timeindex_)
                {
                    condition_->wait(lock);
                }

                timeindex = newest_timeindex_;
            }

            {
                std::unique_lock<osi::Mutex> lock(*mutex_);

                while(newest_timeindex_ < timeindex ||
                      newest_timeindex_ < oldest_timeindex_)
                {
                    condition_->wait(lock);
                }

                if(timeindex < oldest_timeindex_)
                {
                    timeindex = oldest_timeindex_;
                }
                element = (*history_elements_)[timeindex % history_elements_->size()];
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
    int max_length = 1000;

    oldest_timeindex_ = 0;
    newest_timeindex_ = oldest_timeindex_ - 1;


    history_elements_ = std::make_shared<std::vector<double>>(max_length);
    history_timestamps_ = std::make_shared<std::vector<Timestamp>>(max_length);

    condition_ = std::make_shared<osi::ConditionVariable>();
    mutex_ = std::make_shared<osi::Mutex>();



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
