#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <utils/timer.hpp>
#include <utils/os_interface.hpp>

template<typename Type = int> class ThreadsafeTimeseriesInterface
{
public:
    typedef long int Index;
    typedef long double Timestamp;

    virtual Type operator[](Index& desired_timeindex) const = 0;

    virtual Timestamp timestamp(Index& desired_timeindex) const = 0;

    virtual void append(const Type& element) = 0;

    virtual Index next_timeindex() const = 0;

    // waits if empty
    virtual Type current_element() const
    {
        Index timeindex = next_timeindex()-1;
        return (*this)[timeindex];
    }




    virtual size_t history_length() const = 0;
};



template<typename Type=int>
class ThreadsafeTimeseries: public ThreadsafeTimeseriesInterface<Type>
{
public:
    typedef typename ThreadsafeTimeseries<Type>::Index Index;
    typedef typename ThreadsafeTimeseries<Type>::Timestamp Timestamp;


private:
    std::shared_ptr<std::vector<Type>> history_elements_;
    std::shared_ptr<std::vector<Timestamp>> history_timestamps_;

    Index oldest_timeindex_;
    Index newest_timeindex_;

    Index tagged_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:

    ThreadsafeTimeseries(size_t size, Index start_timeindex = 0)
    {
        oldest_timeindex_ = start_timeindex;
        newest_timeindex_ = oldest_timeindex_ - 1;

        tagged_timeindex_ = newest_timeindex_;

        history_elements_ = std::make_shared<std::vector<Type>>(size);
        history_timestamps_ = std::make_shared<std::vector<Timestamp>>(size);

        condition_ = std::make_shared<osi::ConditionVariable>();
        mutex_ = std::make_shared<osi::Mutex>();
    }

    virtual void tag(const Index& timeindex)
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        tagged_timeindex_ = timeindex;
    }

    virtual bool has_changed_since_tag() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return tagged_timeindex_ != newest_timeindex_;
    }

    virtual Index newest_timeindex() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        while(newest_timeindex_ < oldest_timeindex_)
        {
            condition_->wait(lock);
        }

        return newest_timeindex_;
    }

    virtual Type operator[](Index& timeindex) const
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
        Type element
            = (*history_elements_)[timeindex % history_elements_->size()];

        return element;
    }

    virtual Timestamp timestamp(Index& timeindex) const
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
        Timestamp timestamp
            = (*history_timestamps_)[timeindex % history_timestamps_->size()];

        return timestamp;
    }

    virtual void append(const Type& element)
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

    virtual Index next_timeindex() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ + 1;
    }

    virtual size_t history_length() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ - oldest_timeindex_ + 1;
    }
};






