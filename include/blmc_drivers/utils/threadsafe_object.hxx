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

namespace blmc_drivers{

SingletypeThreadsafeObject::SingletypeThreadsafeObject()
{
    // initialize shared pointers ------------------------------------------
    data_ = std::make_shared<std::array<Type, SIZE>>();
    condition_ = std::make_shared<osi::ConditionVariable>();
    condition_mutex_ = std::make_shared<osi::Mutex>();
    modification_counts_ = std::make_shared<std::array<size_t, SIZE>>();
    total_modification_count_ = std::make_shared<size_t>();
    data_mutexes_ = std::make_shared<std::array<osi::Mutex, SIZE>>();

    // initialize counts ---------------------------------------------------
    for(size_t i = 0; i < SIZE; i++)
    {
        (*modification_counts_)[i] = 0;
    }
    *total_modification_count_ = 0;
}

SingletypeThreadsafeObject::SingletypeThreadsafeObject(const std::vector<std::string>& names)
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

SingletypeThreadsafeObject::size_t SingletypeThreadsafeObject::size()
{
    return SingletypeThreadsafeObject::SIZE;
}

SingletypeThreadsafeObject::Type SingletypeThreadsafeObject::get(
  const size_t& index) const
{
    std::unique_lock<osi::Mutex> lock((*data_mutexes_)[index]);
    return (*data_)[index];
}
SingletypeThreadsafeObject::Type SingletypeThreadsafeObject::get(
  const std::string& name) const
{
    return get(name_to_index_.at(name));
}

template<int INDEX=0> 
SingletypeThreadsafeObject::Type SingletypeThreadsafeObject::get() const
{
    return get(INDEX);
}

template<typename Type>
void SingletypeThreadsafeObject::set(
  const Type& datum,
  const size_t& index = 0)
{
    // we sleep for a nanosecond, in case some thread calls set several
    // times in a row. this way we do hopefully not miss messages
    Timer<>::sleep_ms(0.000001);
    // set datum in our data_ member ---------------------------------------
    {
        std::unique_lock<osi::Mutex> lock((*data_mutexes_)[index]);
        (*data_)[index] = datum;
    }

    // notify --------------------------------------------------------------
    {
        std::unique_lock<osi::Mutex> lock(*condition_mutex_);
        (*modification_counts_)[index] += 1;
        *total_modification_count_ += 1;
        condition_->notify_all();
    }
}

/// for backwards compatibility ============================================
template<typename Type, int INDEX=0> void 
SingletypeThreadsafeObject::set(Type datum)
{
    set(datum, INDEX);
}

void SingletypeThreadsafeObject::set(
  const Type& datum,
  const std::string& name)
{
    set(datum, name_to_index_.at(name));
}

void SingletypeThreadsafeObject::wait_for_update(const size_t& index) const
{
    std::unique_lock<osi::Mutex> lock(*condition_mutex_);

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

void SingletypeThreadsafeObject::wait_for_update(
  const std::string& name) const
{
    wait_for_update(name_to_index_.at(name));
}

size_t SingletypeThreadsafeObject::wait_for_update() const
{
    std::unique_lock<osi::Mutex> lock(*condition_mutex_);

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

ThreadsafeObject::ThreadsafeObject()
{
    // initialize shared pointers ------------------------------------------
    data_ = std::make_shared<std::tuple<Types ...> >();
    condition_ = std::make_shared<osi::ConditionVariable>();
    condition_mutex_ = std::make_shared<osi::Mutex>();
    modification_counts_ =
            std::make_shared<std::array<size_t, SIZE>>();
    total_modification_count_ = std::make_shared<size_t>();
    data_mutexes_ = std::make_shared<std::array<osi::Mutex, SIZE>>();

    // initialize counts ---------------------------------------------------
    for(size_t i = 0; i < SIZE; i++)
    {
        (*modification_counts_)[i] = 0;
    }
    *total_modification_count_ = 0;
}

template<int INDEX=0> ThreadsafeObject::Type<INDEX> 
ThreadsafeObject::get() const
{
    std::unique_lock<osi::Mutex> lock((*data_mutexes_)[INDEX]);
    return std::get<INDEX>(*data_);
}

template<int INDEX=0> void ThreadsafeObject::set(Type<INDEX> datum)
{
    // we sleep for a nanosecond, in case some thread calls set several
    // times in a row. this way we do hopefully not miss messages
    Timer<>::sleep_ms(0.000001);
    // set datum in our data_ member ---------------------------------------
    {
        std::unique_lock<osi::Mutex> lock((*data_mutexes_)[INDEX]);
        std::get<INDEX>(*data_) = datum;
    }

    // notify --------------------------------------------------------------
    {
        std::unique_lock<osi::Mutex> lock(*condition_mutex_);
        (*modification_counts_)[INDEX] += 1;
        *total_modification_count_ += 1;
        condition_->notify_all();
    }
}

void ThreadsafeObject::wait_for_update(unsigned index) const
{
    std::unique_lock<osi::Mutex> lock(*condition_mutex_);

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

template< unsigned INDEX=0> void ThreadsafeObject::wait_for_update() const
{
    wait_for_update(INDEX);
}

size_t ThreadsafeObject::wait_for_update() const
{
    std::unique_lock<osi::Mutex> lock(*condition_mutex_);

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