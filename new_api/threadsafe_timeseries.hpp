#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <time_logger.hpp>
#include <os_interface.hpp>

template<typename Type> class ThreadsafeTimeseriesInterface
{
    virtual Type operator[](long int timeindex) const = 0;
    virtual void append(const Type& element) = 0;

    virtual size_t newest_timeindex() const = 0;
    virtual Type newest_element()
    {
        return (*this)[newest_timeindex()];
    }

    virtual size_t size() const = 0;
};



template<typename Type>
class ThreadsafeTimeseries: public ThreadsafeTimeseriesInterface<Type>
{
private:
    std::shared_ptr<std::vector<Type>> history_;

    long int oldest_timeindex_;
    long int newest_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:

    ThreadsafeTimeseries(size_t size, long int start_timeindex = 0)
    {
        oldest_timeindex_ = start_timeindex;
        newest_timeindex_ = oldest_timeindex_ - 1;

        history_ = std::make_shared<std::vector<Type>>(size);
        condition_ = std::make_shared<osi::ConditionVariable>();
        mutex_ = std::make_shared<osi::Mutex>();
    }

    Type operator[](long int timeindex) const
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
            timeindex = oldest_timeindex_;
        }

        return (*history_)[timeindex % history_->size()];
    }

    void append(const Type& element)
    {
        {
            std::unique_lock<osi::Mutex> lock(*mutex_);
            newest_timeindex_++;
            if(newest_timeindex_ - oldest_timeindex_ + 1 > history_->size())
            {
                oldest_timeindex_++;
            }
            (*history_)[newest_timeindex_ % history_->size()] = element;
        }

        condition_->notify_all();
    }

    size_t size() const
    {
        return history_->size();
    }

    size_t newest_timeindex() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_;
    }
};



