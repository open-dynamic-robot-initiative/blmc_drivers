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

    virtual std::tuple<Type, Index>
    operator[](const Index& desired_timeindex) const = 0;

    virtual std::tuple<Timestamp, Index>
    timestamp(const Index& desired_timeindex) const = 0;

    virtual void append(const Type& element) = 0;

    virtual Index next_timeindex() const = 0;

    // waits if empty
    virtual Type current_element() const
    {
        return std::get<0>((*this)[next_timeindex()-1]);
    }



    virtual std::tuple<Type, Index> current_element2() const
    {
        return (*this)[next_timeindex()-1];
    }

    virtual size_t history_length() const = 0;
};



template<typename Type>
class ThreadsafeTimeseries: public ThreadsafeTimeseriesInterface<Type>
{
public:
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

    virtual std::tuple<Type, Index>
    operator[](const Index& desired_timeindex) const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);

        Index timeindex = desired_timeindex;

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

        return std::make_tuple(element, timeindex);
    }

    virtual std::tuple<Timestamp, Index>
    timestamp(const Index& desired_timeindex) const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);

        Index timeindex = desired_timeindex;

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

        return std::make_tuple(timestamp, timeindex);
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




template<typename Type>
class ThreadsafeLoggingTimeseries:
        public ThreadsafeTimeseries<
        std::tuple<Type, ThreadsafeTimeseriesInterface<>::Index>>
{
public:
    typedef ThreadsafeTimeseries<Type> LoggedTimeseries;
    typedef typename ThreadsafeTimeseriesInterface<Type>::Index Index;


    bool has_changed(const LoggedTimeseries& logged_timeseries)
    {
        if(logged_timeseries.history_length() == 0)
        {
            return false;
        }
        if(this->history_length() == 0)
        {
            return true;
        }

        Index current_timeindex = logged_timeseries.next_timeindex() - 1;
        Index logged_timeindex = std::get<1>(this->current_element());

        return (current_timeindex != logged_timeindex);
    }

    void update_if_changed(const LoggedTimeseries& logged_timeseries)
    {
        if(!this->has_changed(logged_timeseries))
        {
            return;
        }

        this->append(logged_timeseries.current_element2());
    }

};



