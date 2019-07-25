/**
 * @file threadsafe_object.hxx
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file defines the functions from threadsafe_object.hpp
 * @version 0.1
 * @date 2018-11-29
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#pragma once

#include "real_time_tools/timer.hpp"

namespace blmc_drivers{

template<typename Type, size_t SIZE>
SingletypeThreadsafeObject<Type, SIZE>::SingletypeThreadsafeObject()
{
    // initialize shared pointers ------------------------------------------
    data_ = std::make_shared<std::array<Type, SIZE>>();
    condition_ = std::make_shared<std::condition_variable>();
    condition_mutex_ = std::make_shared<std::mutex>();
    modification_counts_ = std::make_shared<std::array<size_t, SIZE>>();
    total_modification_count_ = std::make_shared<size_t>();
    data_mutexes_ = std::make_shared<std::array<std::mutex, SIZE>>();

    // initialize counts ---------------------------------------------------
    for(size_t i = 0; i < SIZE; i++)
    {
        (*modification_counts_)[i] = 0;
    }
    *total_modification_count_ = 0;
}

template<typename Type, size_t SIZE>
SingletypeThreadsafeObject<Type, SIZE>::SingletypeThreadsafeObject(
  const std::vector<std::string>& names)
{
    SingletypeThreadsafeObject();
    if(names.size() != size())
    {
        rt_printf("you passed a list of names of wrong size."
                              "expected size: %d, actual size: %d\n",
                              size(), names.size());
        rt_printf("name: %s\n", names[0].c_str());
        exit(-1);
    }

    for(size_t i = 0; i < names.size(); i++)
    {
        name_to_index_[names[i]] = i;
    }
//        name_to_index_ = name_to_index;
}

template<typename Type, size_t SIZE>
void SingletypeThreadsafeObject<Type, SIZE>::set(
  const Type& datum,
  const size_t& index)
{
    // we sleep for a nanosecond, in case some thread calls set several
    // times in a row. this way we do hopefully not miss messages
    real_time_tools::Timer::sleep_sec(1e-9);
    // set datum in our data_ member ---------------------------------------
    {
        std::unique_lock<std::mutex> lock((*data_mutexes_)[index]);
        (*data_)[index] = datum;
    }

    // notify --------------------------------------------------------------
    {
        std::unique_lock<std::mutex> lock(*condition_mutex_);
        (*modification_counts_)[index] += 1;
        *total_modification_count_ += 1;
        condition_->notify_all();
    }
}

template<typename Type, size_t SIZE>
void SingletypeThreadsafeObject<Type, SIZE>::wait_for_update(
  const size_t& index) const
{
    std::unique_lock<std::mutex> lock(*condition_mutex_);

    // wait until the datum with the right index is modified ---------------
    size_t initial_modification_count =
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

template<typename Type, size_t SIZE>
size_t SingletypeThreadsafeObject<Type, SIZE>::wait_for_update() const
{
    std::unique_lock<std::mutex> lock(*condition_mutex_);

    // wait until any datum is modified ------------------------------------
    std::array<size_t, SIZE>
            initial_modification_counts = *modification_counts_;
    size_t initial_modification_count = *total_modification_count_;

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
        else if(initial_modification_counts[i] !=(*modification_counts_)[i])
        {
            rt_printf("something in the threadsafe object "
                      "went horribly wrong\n");
            exit(-1);
        }
    }
    return modified_index;
}

// ========================================================================== // 

template<typename ...Types>
ThreadsafeObject<Types ...>::ThreadsafeObject()
{
    // initialize shared pointers ------------------------------------------
    data_ = std::make_shared<std::tuple<Types ...> >();
    condition_ = std::make_shared<std::condition_variable>();
    condition_mutex_ = std::make_shared<std::mutex>();
    modification_counts_ =
            std::make_shared<std::array<size_t, SIZE>>();
    total_modification_count_ = std::make_shared<size_t>();
    data_mutexes_ = std::make_shared<std::array<std::mutex, SIZE>>();

    // initialize counts ---------------------------------------------------
    for(size_t i = 0; i < SIZE; i++)
    {
        (*modification_counts_)[i] = 0;
    }
    *total_modification_count_ = 0;
}

template<class ... Types>
template<int INDEX>
ThreadsafeObject<Types ...>::Type<INDEX> ThreadsafeObject<Types ...>::get() const
{
    std::unique_lock<std::mutex> lock((*data_mutexes_)[INDEX]);
    return std::get<INDEX>(*data_);
}

template<class ... Types>
template<int INDEX> void
ThreadsafeObject<Types ...>::set(ThreadsafeObject<Types ...>::Type<INDEX> datum)
{
    // we sleep for a nanosecond, in case some thread calls set several
    // times in a row. this way we do hopefully not miss messages
    real_time_tools::Timer::sleep_sec(1e-9);
    // set datum in our data_ member ---------------------------------------
    {
        std::unique_lock<std::mutex> lock((*data_mutexes_)[INDEX]);
        std::get<INDEX>(*data_) = datum;
    }

    // notify --------------------------------------------------------------
    {
        std::unique_lock<std::mutex> lock(*condition_mutex_);
        (*modification_counts_)[INDEX] += 1;
        *total_modification_count_ += 1;
        condition_->notify_all();
    }
}

template<class ... Types>
void ThreadsafeObject<Types ...>::wait_for_update(unsigned index) const
{
    std::unique_lock<std::mutex> lock(*condition_mutex_);

    // wait until the datum with the right index is modified ---------------
    size_t initial_modification_count =
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

template<class ... Types>
size_t ThreadsafeObject<Types ...>::wait_for_update() const
{
    std::unique_lock<std::mutex> lock(*condition_mutex_);

    // wait until any datum is modified ------------------------------------
    std::array<size_t, SIZE>
            initial_modification_counts = *modification_counts_;
    size_t initial_modification_count = *total_modification_count_;

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
        else if(initial_modification_counts[i] !=(*modification_counts_)[i])
        {
            rt_printf("something in the threadsafe object "
                      "went horribly wrong\n");
            exit(-1);
        }
    }
    return modified_index;
}

} // namespace blmc_drivers