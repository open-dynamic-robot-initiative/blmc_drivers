/**
 * @file threadsafe_object.hpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief This file declares templated container for data buffering
 * @version 0.1
 * @date 2018-11-27
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>
#include <vector>

#include "real_time_tools/timer.hpp"

#include "blmc_drivers/utils/os_interface.hpp"

namespace blmc_drivers{

/**
 * @brief This is a template abstract interface class that define a data history.
 * This re-writting of the vector style class is thread safe. So it allows the
 * user to use the object without having to deal with mutexes nor condition
 * variables. This class is not used so far.
 * 
 * @tparam Type is the type of the data to store.
 */
template<typename Type> class ThreadsafeHistoryInterface
{
    /**
     * @brief Get the element after the one with the given id. if there is no
     * newer element, then wait until one arrives.
     * 
     * @param id is the index of the element in the buffer.
     * @return Type the next element.
     */
    virtual Type get_next(size_t id) const
    {
        return get(get_next_id(id));
    }
    virtual size_t get_next_id(size_t id) const = 0;

    /**
     * @brief Get the newest value, this function waits if it is empty.
     * 
     * @return Type the newest element.
     */
    virtual Type get_newest() const
    {
        return get(get_newest_id());
    }

    /**
     * @brief get_newest_id
     * 
     * @return size_t 
     */
    virtual size_t get_newest_id() const = 0;

    /**
     * @brief Get the value whith a specific id.
     * 
     * @param id 
     * @return Type 
     */
    virtual Type get(size_t id) const = 0;

    /**
     * @brief I guess this is to add a value but with no argument?
     * \todo Manuel, could you delete this class or provide an implementation?
     */
    virtual void add() = 0;
};

/**
 * @brief The SingletypeThreadsafeObject is a thread safe object 
 * 
 * @tparam Type is the data type to store in the buffer.
 * @tparam SIZE is the size of the buffer. It is better to know it at compile
 * time to be 100% real time safe.
 */
template<typename Type, size_t SIZE> class SingletypeThreadsafeObject
{
public:
    /**
     * @brief Construct a new SingletypeThreadsafeObject object
     */
    SingletypeThreadsafeObject();

    /**
     * @brief Construct a new SingletypeThreadsafeObject object
     * 
     * @param names 
     */
    SingletypeThreadsafeObject(const std::vector<std::string>& names);

    /**
     * @brief Wait until the data at the given index is modified.
     * 
     * @param index 
     */
    void wait_for_update(const size_t& index) const;

    /**
     * @brief Wait until the data at the given name is modified.
     * 
     * @param name 
     */
    void wait_for_update(const std::string& name) const
    {
        wait_for_update(name_to_index_.at(name));
    }

    /**
     * @brief Wait unitl any data has been changed and return its index.
     * 
     * @return size_t 
     */
    size_t wait_for_update() const;

    /**
     * Getters
     */

    /**
     * @brief get size.
     * 
     * @return size_t 
     */
    size_t size()
    {
        return SIZE;
    }

    /**
     * @brief Get the data by its index in the buffer.
     * 
     * @param index 
     * @return Type 
     */
    Type get(const size_t& index = 0) const
    {
        std::unique_lock<osi::Mutex> lock((*data_mutexes_)[index]);
        return (*data_)[index];
    }

    /**
     * @brief Get the data by its name in the buffer.
     * 
     * @param name 
     * @return Type 
     */
    Type get(const std::string& name) const
    {
        return get(name_to_index_.at(name));
    }

    /**
     * @brief Get the data by its index in the buffer. Index is solved during
     * compile time
     * 
     * @tparam INDEX=0 
     * @return Type 
     */
    template<int INDEX=0> Type get() const
    {
        return get(INDEX);
    }

    /**
     * Setters.
     */

    /**
     * @brief Set one element at a designated index.
     * 
     * @param datum 
     * @param index 
     */
    void set(const Type& datum, const size_t& index = 0);

    /**
     * @brief Set one element at a designated index.
     * Warning the index is resolved at compile time.
     * This is used for backward comaptibility.
     * \todo "This is used for backward comaptibility.", Manuel Which bakward?
     * 
     * @tparam INDEX=0 
     * @param datum 
     */
    template<int INDEX=0> void set(Type datum)
    {
        set(datum, INDEX);
    }

    /**
     * @brief Set one element using at a designated name. Internally this name
     * is map to an index.
     * 
     * @param datum 
     * @param name 
     */
    void set(const Type& datum, const std::string& name)
    {
        set(datum, name_to_index_.at(name));
    }

private:
    /**
     * @brief This is the data buffer.
     */
    std::shared_ptr<std::array<Type, SIZE> > data_;
    /**
     * @brief This is counting the data modification occurences for each
     * individual buffers.
     */
    std::shared_ptr<std::array<size_t, SIZE>> modification_counts_;
    /**
     * @brief This is counting the all data modification occurences for all
     * buffer.
     * /todo Can't we just some the modification_counts_ array whenever needed?
     */
    std::shared_ptr<size_t> total_modification_count_;

    /**
     * @brief This is the map that allow to deal with data by their names.
     */
    std::map<std::string, size_t> name_to_index_;

    /**
     * @brief This condition variable is used to wait untils any data has been
     * changed.
     */
    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    /**
     * @brief This is the mutex of the condition varaible.
     */
    mutable std::shared_ptr<osi::Mutex> condition_mutex_;
    /**
     * @brief These are the individual mutexes of each data upon setting and
     * getting.
     */
    mutable std::shared_ptr<std::array<osi::Mutex, SIZE>> data_mutexes_;
};

/**
 * @brief This object can have several types depending on what ones want to
 * store.
 * 
 * @tparam Types 
 */
template<typename ...Types> class ThreadsafeObject
{
public:
    /**
     * @brief Define a specific "Type" which permit a more readable code.
     * 
     * @tparam INDEX 
     */
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    /**
     * @brief Define the size of the different types. 
     */
    static const std::size_t SIZE = sizeof...(Types);

    /**
     * @brief Construct a new ThreadsafeObject object
     */
    ThreadsafeObject();

    /**
     * @brief Wait until the data with the deignated index is changed.
     * 
     * @param index 
     */
    void wait_for_update(unsigned index) const;

    /**
     * @brief Wait until the data with the designated index is changed.
     * 
     * @tparam INDEX=0 
     */
    template< unsigned INDEX=0> void wait_for_update() const
    {
        wait_for_update(INDEX);
    }

    /**
     * @brief Wait until any data has been changed.
     * 
     * @return size_t 
     */
    size_t wait_for_update() const;

    /**
     * Getters
     */

    /**
     * @brief Get the data with the designated index. The index is resolved at
     * compile time.
     * 
     * @tparam INDEX=0 
     * @return Type<INDEX> 
     */
    template<int INDEX=0> Type<INDEX> get() const;
    
    /**
     * Setters
     */

    /**
     * @brief Set the data with the designated index. The index is resolved at
     * compile time.
     * 
     * @tparam INDEX=0 
     * @param datum 
     */
    template<int INDEX=0> void set(Type<INDEX> datum);

private:
    /**
     * @brief the actual data buffers.
     */
    std::shared_ptr<std::tuple<Types ...> > data_;
    /**
     * @brief a condition variable that allow to wait until one data has been
     * changed in the buffer.
     */
    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    /**
     * @brief The mutex of the condition variable.
     */
    mutable std::shared_ptr<osi::Mutex> condition_mutex_;
    /**
     * @brief This is counting the data modification occurences for each
     * individual buffers.
     */
    std::shared_ptr<std::array<size_t, SIZE>> modification_counts_;
    /**
     * @brief This is counting the all data modification occurences for all
     * buffer.
     * /todo Can't we just some the modification_counts_ array whenever needed?
     */
    std::shared_ptr<size_t> total_modification_count_;  
    /**
     * @brief These are the individual mutexes of each data upon setting and
     * getting.
     */
    std::shared_ptr<std::array<osi::Mutex, SIZE>> data_mutexes_;
};

} // namespace blmc_drivers

#include "blmc_drivers/utils/threadsafe_timeseries.hxx"
