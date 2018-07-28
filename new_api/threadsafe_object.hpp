#pragma once


#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/cond.h>


#include <mutex>
#include <condition_variable>

#include <rtdk.h>


#include <array>
#include <tuple>

#include <memory>






template<typename ThreadsafeInput, typename ThreadsafeOutput> class InputOutputObject
{
public:
    InputOutputObject() { }
    virtual ~InputOutputObject() { }

    virtual std::shared_ptr<ThreadsafeInput> get_input() = 0;

    virtual std::shared_ptr<const ThreadsafeInput> get_output() = 0;


};


namespace xenomai
{
class mutex
{
public:
    RT_MUTEX rt_mutex_;

    mutex()
    {
        rt_mutex_create(&rt_mutex_, NULL);
    }

    void lock()
    {
        rt_mutex_acquire(&rt_mutex_, TM_INFINITE);

    }

    void unlock()
    {
        rt_mutex_release(&rt_mutex_);
    }

};


class condition_variable
{
public:
    RT_COND rt_condition_variable_;


    condition_variable()
    {
        rt_cond_create(&rt_condition_variable_, NULL);
    }

    void wait(std::unique_lock<mutex>& lock )
    {
        rt_cond_wait(&rt_condition_variable_, &lock.mutex()->rt_mutex_, TM_INFINITE);
    }

    void notify_all()
    {
        rt_cond_broadcast(&rt_condition_variable_);
    }
};
}


template<typename ...Types> class ThreadsafeObject
{
public:
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    static const std::size_t SIZE = sizeof...(Types);

#ifdef __XENO__
    typedef xenomai::mutex Mutex;
    typedef xenomai::condition_variable ConditionVariable;
#else
    typedef std::mutex Mutex;
    typedef std::condition_variable ConditionVariable;
#endif

private:

    std::shared_ptr<std::tuple<Types ...> > data_;

    mutable std::shared_ptr<ConditionVariable> condition_;
    mutable std::shared_ptr<Mutex> condition_mutex_;
    std::shared_ptr<std::array<long unsigned, SIZE>> modification_counts_;
    std::shared_ptr<long unsigned> total_modification_count_;

    std::shared_ptr<std::array<Mutex, SIZE>> data_mutexes_;


public:
    ThreadsafeObject()
    {
        // initialize shared pointers ------------------------------------------
        data_ = std::make_shared<std::tuple<Types ...> >();
        condition_ = std::make_shared<ConditionVariable>();
        condition_mutex_ = std::make_shared<Mutex>();
        modification_counts_ =
                std::make_shared<std::array<long unsigned, SIZE>>();
        total_modification_count_ = std::make_shared<long unsigned>();
        data_mutexes_ = std::make_shared<std::array<Mutex, SIZE>>();

        // initialize counts ---------------------------------------------------
        for(size_t i = 0; i < SIZE; i++)
        {
            (*modification_counts_)[i] = 0;
        }
        *total_modification_count_ = 0;
    }

    template<int INDEX> Type<INDEX> get()
    {
        std::unique_lock<Mutex> lock((*data_mutexes_)[INDEX]);
        return std::get<INDEX>(*data_);
    }

    template<int INDEX> void set(Type<INDEX> datum)
    {
        // set datum in our data_ member ---------------------------------------
        {
            std::unique_lock<Mutex> lock((*data_mutexes_)[INDEX]);
            std::get<INDEX>(*data_) = datum;
        }

        // notify --------------------------------------------------------------
        {
            std::unique_lock<Mutex> lock(*condition_mutex_);
            (*modification_counts_)[INDEX] += 1;
            *total_modification_count_ += 1;
            condition_->notify_all();
        }
    }

    void wait_for_datum(unsigned index)
    {
        std::unique_lock<Mutex> lock(*condition_mutex_);

        // wait until the datum with the right index is modified ---------------
        long unsigned initial_modification_count =
                (*modification_counts_)[index];
        while(initial_modification_count == (*modification_counts_)[index])
        {
            condition_->wait(lock);
        }

        // check that we did not miss data -------------------------------------
        if(initial_modification_count + 1 != (*modification_counts_)[index])
        {
            rt_printf("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );
            rt_printf("something went wrong, we missed a message.\n");
            exit(-1);
        }
    }

    long unsigned wait_for_datum()
    {
        std::unique_lock<Mutex> lock(*condition_mutex_);

        // wait until any datum is modified ------------------------------------
        std::array<long unsigned, SIZE>
                initial_modification_counts = *modification_counts_;
        long unsigned initial_modification_count = *total_modification_count_;

        while(initial_modification_count == *total_modification_count_)
        {
            condition_->wait(lock);
        }

        // make sure we did not miss any data ----------------------------------
        if(initial_modification_count + 1 != *total_modification_count_)
        {
            rt_printf("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );

            rt_printf("something went wrong, we missed a message.\n");
            exit(-1);
        }

        // figure out which index was modified and return it -------------------
        int modified_index = -1;
        for(size_t i = 0; i < SIZE; i++)
        {
            if(initial_modification_counts[i] + 1 == (*modification_counts_)[i])
            {
                if(modified_index != -1)
                {
                    rt_printf("something in the threadsafe object "
                              "went horribly wrong\n");
                    exit(-1);
                }

                modified_index = i;
            }
            else if(initial_modification_counts[i] != (*modification_counts_)[i])
            {
                rt_printf("something in the threadsafe object "
                          "went horribly wrong\n");
                exit(-1);
            }
        }
        return modified_index;
    }


};
