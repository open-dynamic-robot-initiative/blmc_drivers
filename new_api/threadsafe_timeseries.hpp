#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <time_logger.hpp>
#include <os_interface.hpp>

template<typename Type> class ThreadsafeTimeseriesInterface
{
    // return the element after the one with the given id. if there is no
    // newer element, then wait until one arrives.
    virtual Type get_next(size_t id) const
    {
        return get(get_next_id(id));
    }
    virtual size_t get_next_id(size_t id) const = 0;

    // wait if empty
    virtual Type get_newest() const
    {
        return get(get_newest_id());
    }
    virtual size_t get_newest_id() const = 0;

    virtual Type get(size_t id) const = 0;

    virtual void add() = 0;
};



template<typename Type> class ThreadsafeTimeseries
{
private:
    std::shared_ptr<std::vector<Type>> history_;

    int oldest_timeindex_;
    int newest_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:

    ThreadsafeTimeseries(size_t size, int start_timeindex = 0)
    {
        oldest_timeindex_ = start_timeindex;
        newest_timeindex_ = oldest_timeindex_ - 1;

        history_ = std::make_shared<std::vector<Type>>(size);
        condition_ = std::make_shared<osi::ConditionVariable>();
        mutex_ = std::make_shared<osi::Mutex>();
    }

    Type operator[](int timeindex) const
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
                                 "history (anymore). returning oldest existing"
                                 "element.\n");
            timeindex = oldest_timeindex_;
        }

        return (*history_)[timeindex % history_->size()];
    }

    size_t size()
    {
        return history_->size();
    }

    void append(const Type& element)
    {
        {
            std::unique_lock<osi::Mutex> lock(*mutex_);
            newest_timeindex_++;
            if(newest_timeindex_ - oldest_timeindex_ > history_->size())
            {
                oldest_timeindex_++;
            }
            (*history_)[newest_timeindex_ % history_->size()] = element;
        }

        condition_->notify_all();
    }
};



