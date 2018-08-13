#pragma once

#include <array>
#include <tuple>
#include <memory>
#include <map>

#include <utils/timer.hpp>
#include <utils/os_interface.hpp>

template<typename Type> class ThreadsafeHistoryInterface
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



template<typename Type, size_t SIZE> class SingletypeThreadsafeObject
{
private:
    std::shared_ptr<std::array<Type, SIZE> > data_;
    std::shared_ptr<std::array<size_t, SIZE>> modification_counts_;
    std::shared_ptr<size_t> total_modification_count_;

    std::map<std::string, size_t> name_to_index_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> condition_mutex_;
    mutable std::shared_ptr<std::array<osi::Mutex, SIZE>> data_mutexes_;

public:
    SingletypeThreadsafeObject()
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

    SingletypeThreadsafeObject(const std::vector<std::string>& names):
        SingletypeThreadsafeObject()
    {
        if(names.size() != size())
        {
            osi::print_to_screen("you passed a list of names of wrong size."
                                 "expected size: %d, actual size: %d\n",
                                 size(), names.size());
            osi::print_to_screen("name: %s\n", names[0].c_str());
            exit(-1);
        }

        for(size_t i = 0; i < names.size(); i++)
        {
            name_to_index_[names[i]] = i;
        }
//        name_to_index_ = name_to_index;
    }

    size_t size()
    {
        return SIZE;
    }

    Type get(const size_t& index = 0) const
    {
        std::unique_lock<osi::Mutex> lock((*data_mutexes_)[index]);
        return (*data_)[index];
    }
    Type get(const std::string& name) const
    {
        return get(name_to_index_.at(name));
    }

    /// for backwards compatibility ============================================
    template<int INDEX=0> void set(Type datum)
    {
        set(datum, INDEX);
    }

    template<int INDEX=0> Type get() const
    {
        return get(INDEX);
    }


    /// ========================================================================

    void set(const Type& datum, const size_t& index = 0)
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
    void set(const Type& datum, const std::string& name)
    {
        set(datum, name_to_index_.at(name));
    }

    void wait_for_update(const size_t& index) const
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
            osi::print_to_screen("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );
            osi::print_to_screen("something went wrong, we missed a message.\n");
            exit(-1);
        }
    }
    void wait_for_update(const std::string& name) const
    {
        wait_for_update(name_to_index_.at(name));
    }

    size_t wait_for_update() const
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
            osi::print_to_screen("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );

            osi::print_to_screen("something went wrong, we missed a message.\n");
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
                    osi::print_to_screen("something in the threadsafe object "
                              "went horribly wrong\n");
                    exit(-1);
                }

                modified_index = i;
            }
            else if(initial_modification_counts[i] !=(*modification_counts_)[i])
            {
                osi::print_to_screen("something in the threadsafe object "
                          "went horribly wrong\n");
                exit(-1);
            }
        }
        return modified_index;
    }
};





template<typename ...Types> class ThreadsafeObject
{
public:
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    static const std::size_t SIZE = sizeof...(Types);

private:
    std::shared_ptr<std::tuple<Types ...> > data_;

    mutable std::shared_ptr<osi::ConditionVariable> condition_;
    mutable std::shared_ptr<osi::Mutex> condition_mutex_;
    std::shared_ptr<std::array<size_t, SIZE>> modification_counts_;
    std::shared_ptr<size_t> total_modification_count_;

    std::shared_ptr<std::array<osi::Mutex, SIZE>> data_mutexes_;

public:
    ThreadsafeObject()
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

    template<int INDEX=0> Type<INDEX> get() const
    {
        std::unique_lock<osi::Mutex> lock((*data_mutexes_)[INDEX]);
        return std::get<INDEX>(*data_);
    }

    template<int INDEX=0> void set(Type<INDEX> datum)
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

    void wait_for_update(unsigned index) const
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
            osi::print_to_screen("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );
            osi::print_to_screen("something went wrong, we missed a message.\n");
            exit(-1);
        }
    }

    template< unsigned INDEX=0> void wait_for_update() const
    {
        wait_for_update(INDEX);
    }

    size_t wait_for_update() const
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
            osi::print_to_screen("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );

            osi::print_to_screen("something went wrong, we missed a message.\n");
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
                    osi::print_to_screen("something in the threadsafe object "
                              "went horribly wrong\n");
                    exit(-1);
                }

                modified_index = i;
            }
            else if(initial_modification_counts[i] !=(*modification_counts_)[i])
            {
                osi::print_to_screen("something in the threadsafe object "
                          "went horribly wrong\n");
                exit(-1);
            }
        }
        return modified_index;
    }
};
