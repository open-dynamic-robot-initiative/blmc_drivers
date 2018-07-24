#pragma once


#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/cond.h>

#include <rtdk.h>


#include <array>
#include <tuple>









template<typename ...Types> class ThreadsafeObject
{
public:
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    static const std::size_t SIZE = sizeof...(Types);

private:

    std::tuple<Types ...> data_;

    mutable RT_COND condition_;
    mutable RT_MUTEX condition_mutex_;
    std::array<long unsigned, SIZE> modification_counts_;
    long unsigned total_modification_count_;

    std::array<RT_MUTEX, SIZE> data_mutexes_;


public:
    ThreadsafeObject()
    {
        rt_cond_create(&condition_, NULL);
        rt_mutex_create(&condition_mutex_, NULL);

        for(size_t i = 0; i < SIZE; i++)
        {
            rt_mutex_create(&data_mutexes_[i], NULL);
            modification_counts_[i] = 0;
        }
        total_modification_count_ = 0;
    }

    template<int INDEX> Type<INDEX> get()
    {
        rt_mutex_acquire(&data_mutexes_[INDEX], TM_INFINITE);
        Type<INDEX> datum = std::get<INDEX>(data_);
        rt_mutex_release(&data_mutexes_[INDEX]);

        return datum;
    }

    template<int INDEX> void set(Type<INDEX> datum)
    {
        rt_mutex_acquire(&data_mutexes_[INDEX], TM_INFINITE);
        std::get<INDEX>(data_) = datum;
        rt_mutex_release(&data_mutexes_[INDEX]);

        // this is a bit suboptimal since we always broadcast on the same condition
        rt_mutex_acquire(&condition_mutex_, TM_INFINITE);
        modification_counts_[INDEX] += 1;
        total_modification_count_ += 1;
        rt_cond_broadcast(&condition_);
        rt_mutex_release(&condition_mutex_);
    }

    void wait_for_datum(unsigned index)
    {
        rt_mutex_acquire(&condition_mutex_, TM_INFINITE);
        long unsigned initial_modification_count = modification_counts_[index];

        while(initial_modification_count == modification_counts_[index])
        {
            rt_cond_wait(&condition_, &condition_mutex_, TM_INFINITE);
        }

        if(initial_modification_count + 1 != modification_counts_[index])
        {
            rt_printf("something went wrong, we missed a message.");
            rt_printf("initial_modification_count: %d, current modification count: %d\n",
                      initial_modification_count, modification_counts_[index]);
            exit(-1);
        }

        rt_mutex_release(&condition_mutex_);
    }

    long unsigned wait_for_datum()
    {
        rt_mutex_acquire(&condition_mutex_, TM_INFINITE);
        long unsigned initial_modification_count = total_modification_count_;

        while(initial_modification_count == total_modification_count_)
        {
            rt_cond_wait(&condition_, &condition_mutex_, TM_INFINITE);
        }

        if(initial_modification_count + 1 != total_modification_count_)
        {
            rt_printf("something went wrong, we missed a message.");
            rt_printf("initial_modification_count: %d, current modification count: %d\n",
                      initial_modification_count, total_modification_count_);
            exit(-1);
        }

        rt_mutex_release(&condition_mutex_);
    }


private:

};
