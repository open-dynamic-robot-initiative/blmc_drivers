#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <time_logger.hpp>
#include <os_interface.hpp>

template<typename Type> class ThreadsafeTimeseriesInterface
{
public:
    typedef long int Index;

    virtual Type operator[](Index timeindex) const = 0;
    virtual void append(const Type& element) = 0;

    // these two functions return immediately. if called on object of zero size,
    // undefined behaviour
    virtual Index next_timeindex() const = 0;
//    virtual Index current_timeindex() const = 0;




    // waits if empty
    virtual Type current_element()
    {
        return (*this)[next_timeindex()-1];
    }

    virtual size_t size() const = 0;
};



template<typename Type>
class ThreadsafeTimeseries: public ThreadsafeTimeseriesInterface<Type>
{
    typedef typename ThreadsafeTimeseriesInterface<Type>::Index Index;

private:
    std::shared_ptr<std::vector<Type>> history_;

    Index oldest_timeindex_;
    Index newest_timeindex_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> mutex_;

public:

    ThreadsafeTimeseries(size_t size, Index start_timeindex = 0)
    {
        oldest_timeindex_ = start_timeindex;
        newest_timeindex_ = oldest_timeindex_ - 1;

        history_ = std::make_shared<std::vector<Type>>(size);
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
            /// \todo we will get rid of this exit, just for now to not miss
            /// this case
            exit(-1);
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
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ - oldest_timeindex_ + 1;
    }

    Index newest_timeindex() const
    {
        if(size() == 0)
            return std::numeric_limits<Index>::quiet_NaN();
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_;
    }

    Index next_timeindex() const
    {
        std::unique_lock<osi::Mutex> lock(*mutex_);
        return newest_timeindex_ + 1;
    }
};



