#pragma once

#include <memory>
#include <vector>

#include <utils/os_interface.hpp>
#include <utils/timer.hpp>


template<typename Type=int> class ThreadsafeTimeseries
{
public:
    typedef long int Index;
    typedef long double Timestamp;

private:
    std::shared_ptr<std::vector<Type>> history_elements_;
    std::shared_ptr<std::vector<Timestamp>> history_timestamps_;

    Index oldest_timeindex_;
    Index newest_timeindex_;
    Index tagged_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:
    ThreadsafeTimeseries(size_t size, Index start_timeindex = 0);

    // accessors ---------------------------------------------------------------
    virtual bool has_changed_since_tag() const;

    virtual Index newest_timeindex() const;
    virtual Index next_timeindex() const;
    virtual Type newest_element() const;

    virtual Type operator[](Index& timeindex) const;
    virtual Timestamp timestamp(Index& timeindex) const;

    virtual size_t history_length() const;

    // mutators ----------------------------------------------------------------
    virtual void tag(const Index& timeindex);
    virtual void append(const Type& element);
};




template<typename Type> ThreadsafeTimeseries<Type>::
ThreadsafeTimeseries(size_t size, Index start_timeindex)
{
    oldest_timeindex_ = start_timeindex;
    newest_timeindex_ = oldest_timeindex_ - 1;

    tagged_timeindex_ = newest_timeindex_;

    history_elements_ = std::make_shared<std::vector<Type>>(size);
    history_timestamps_ = std::make_shared<std::vector<Timestamp>>(size);

    condition_ = std::make_shared<osi::ConditionVariable>();
    mutex_ = std::make_shared<osi::Mutex>();
}

template<typename Type>
void ThreadsafeTimeseries<Type>::tag(const Index& timeindex)
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    tagged_timeindex_ = timeindex;
}

template<typename Type>
bool ThreadsafeTimeseries<Type>::has_changed_since_tag() const
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    return tagged_timeindex_ != newest_timeindex_;
}

template<typename Type> typename ThreadsafeTimeseries<Type>::Index
ThreadsafeTimeseries<Type>::newest_timeindex() const
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    while(newest_timeindex_ < oldest_timeindex_)
    {
        condition_->wait(lock);
    }

    return newest_timeindex_;
}

template<typename Type> Type
ThreadsafeTimeseries<Type>::newest_element() const
{
    Index timeindex = next_timeindex()-1;
    return (*this)[timeindex];
}

template<typename Type> Type
ThreadsafeTimeseries<Type>::operator[](Index& timeindex) const
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

template<typename Type> typename ThreadsafeTimeseries<Type>::Timestamp
ThreadsafeTimeseries<Type>::timestamp(Index& timeindex) const
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

template<typename Type>
void ThreadsafeTimeseries<Type>::append(const Type& element)
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

template<typename Type> typename ThreadsafeTimeseries<Type>::Index
ThreadsafeTimeseries<Type>::next_timeindex() const
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    return newest_timeindex_ + 1;
}

template<typename Type>
size_t ThreadsafeTimeseries<Type>::history_length() const
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    return newest_timeindex_ - oldest_timeindex_ + 1;
}

