#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <utils/timer.hpp>
#include <utils/os_interface.hpp>

template<typename Type> class ThreadsafeTimeseriesInterface
{
public:
    typedef long int Index;
    typedef long double Timestamp;

    virtual Type operator[](Index timeindex) const = 0;

    virtual Timestamp timestamp(Index timeindex) const = 0;

    virtual void append(const Type& element) = 0;

    virtual Index next_timeindex() const = 0;

    // waits if empty
    virtual Type current_element() const
    {
        return (*this)[next_timeindex()-1];
    }

    virtual size_t history_length() const = 0;
};



template<typename Type>
class ThreadsafeTimeseries: public ThreadsafeTimeseriesInterface<Type>
{
    typedef typename ThreadsafeTimeseriesInterface<Type>::Index Index;
    typedef typename ThreadsafeTimeseriesInterface<Type>::Timestamp Timestamp;


private:
    std::shared_ptr<std::vector<Type>> history_elements_;
    std::shared_ptr<std::vector<Timestamp>> history_timestamps_;

    Index oldest_timeindex_;
    Index newest_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:

    ThreadsafeTimeseries(size_t size, Index start_timeindex = 0)
    {
        oldest_timeindex_ = start_timeindex;
        newest_timeindex_ = oldest_timeindex_ - 1;

        history_elements_ = std::make_shared<std::vector<Type>>(size);
        history_timestamps_ = std::make_shared<std::vector<Timestamp>>(size);

        condition_ = std::make_shared<osi::ConditionVariable>();
        mutex_ = std::make_shared<osi::Mutex>();
    }

    Type operator[](Index timeindex) const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);

        while(newest_timeindex_ < timeindex ||
              newest_timeindex_ < oldest_timeindex_)
        {
            condition_->wait(lock);
        }

        if(timeindex < oldest_timeindex_)
        {
            osi::print_to_screen("WARNING: you are trying to access a "
                                 "timeseries element which is not in our "
                                 "history (anymore). returning oldest existing "
                                 "element.\n");

            osi::print_to_screen("some info: %s \n", __PRETTY_FUNCTION__ );
            /// \todo we will get rid of this exit, just for now to not miss
            /// this case
            exit(-1);
            timeindex = oldest_timeindex_;
        }

        return (*history_elements_)[timeindex % history_elements_->size()];
    }
    Timestamp timestamp(Index timeindex) const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);

        while(newest_timeindex_ < timeindex ||
              newest_timeindex_ < oldest_timeindex_)
        {
            condition_->wait(lock);
        }

        if(timeindex < oldest_timeindex_)
        {
            osi::print_to_screen("WARNING: you are trying to access a "
                                 "timeseries element which is not in our "
                                 "history (anymore). returning oldest existing "
                                 "element.\n");

            osi::print_to_screen("some info: %s \n", __PRETTY_FUNCTION__ );
            /// \todo we will get rid of this exit, just for now to not miss
            /// this case
            exit(-1);
            timeindex = oldest_timeindex_;
        }

        return (*history_timestamps_)[timeindex % history_timestamps_->size()];
    }


    void append(const Type& element)
    {
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
    }

    Index next_timeindex() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ + 1;
    }

    size_t history_length() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ - oldest_timeindex_ + 1;
    }
};



