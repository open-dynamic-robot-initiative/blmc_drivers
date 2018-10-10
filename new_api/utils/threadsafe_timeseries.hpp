#pragma once

#include <memory>
#include <vector>

#include <utils/os_interface.hpp>
#include <utils/timer.hpp>



/*! \brief implements a timeseries  \f$ X_{{oldest}:{newest}} \f$ which can safely
 * be accessed from multiple threads.
 *
 * this object has the following properties:
 * - an oldest timeindex \f$ oldest\f$,
 * - a newest timeindex \f$ newest \f$,
 * - a value \f$ X_i \f$ for each  \f$ i \in \{oldest, oldest + 1 , ..., newest\} \f$,
 * - a length \f$length\f$
 * - and a maximum length \f$maxlength\f$
 */
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
    /*! \brief initializes to an empty timeseries with the given
     * \f$ maxlength \f$. when the first element will be inserted, it will
     * have index start_timeindex.
     *
     * this means that this timeseries contains no elements yet,
     * hence \f$ newest \f$ and \f$ oldest \f$ are not defined after
     * construction, there exist no elements and \f$length\f$ is zero
     */
    ThreadsafeTimeseries(size_t max_length, Index start_timeindex = 0);

    // accessors ---------------------------------------------------------------
    /*! \brief returns \f$ newest \f$. waits if the timeseries is empty.
     */
    virtual Index newest_timeindex() const;

    /*! \brief returns \f$ X_{newest} \f$. waits if the timeseries is empty.
     */
    virtual Type newest_element() const;

    /*! \brief returns \f$ X_{timeindex} \f$. waits if the timeseries is empty
     * or if \f$timeindex > newest \f$. if \f$timeindex < oldest \f$ it
     * returns \f$X_{oldest}\f$.
     */
    virtual Type operator[](Index& timeindex) const;

    /*! \brief returns the time in miliseconds when \f$ X_{timeindex} \f$
     * was appended
     */
    virtual Timestamp timestamp_ms(Index& timeindex) const;

    /*! \brief returns the length of the timeseries, i.e. \f$0\f$ if it is
     * empty, otherwise \f$newest - oldest +1 \f$.
     */
    virtual size_t length() const;

    /*! \brief returns boolean indicating whether new elements have been
     * appended since the last time the tag() function was called.
     */
    virtual bool has_changed_since_tag() const;

    // mutators ----------------------------------------------------------------
    /*! \brief tags the current timeseries, can later be used to check
     * whether new elements have been added
     */
    virtual void tag(const Index& timeindex);

    /*! \brief appends a new element to the timeseries, e.g. we go from
     * \f$ X_{1:10} \f$ to \f$ X_{1:11} \f$ (where \f$ X_{11}=\f$ element).
     * if the timeseries length is
     * already equal to its max_length, then the oldest element is discarded,
     * e.g. for a max_length = 10 we would go from \f$ X_{1:10} \f$
     * to \f$ X_{2:11} \f$.
     */
    virtual void append(const Type& element);
};




template<typename Type> ThreadsafeTimeseries<Type>::
ThreadsafeTimeseries(size_t max_length, Index start_timeindex)
{
    oldest_timeindex_ = start_timeindex;
    newest_timeindex_ = oldest_timeindex_ - 1;

    tagged_timeindex_ = newest_timeindex_;

    history_elements_ = std::make_shared<std::vector<Type>>(max_length);
    history_timestamps_ = std::make_shared<std::vector<Timestamp>>(max_length);

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
    Index timeindex = newest_timeindex();
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
ThreadsafeTimeseries<Type>::timestamp_ms(Index& timeindex) const
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

template<typename Type>
size_t ThreadsafeTimeseries<Type>::length() const
{
    std::unique_lock<osi::Mutex> lock(*mutex_);
    return newest_timeindex_ - oldest_timeindex_ + 1;
}
