#pragma once


#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/cond.h>


#include <mutex>
#include <condition_variable>

#include <rtdk.h>


#include <array>
#include <tuple>

#include <memory>






template<typename ThreadsafeInput, typename ThreadsafeOutput>
class InputOutputObject
{
public:
    InputOutputObject() { }
    virtual ~InputOutputObject() { }

    virtual std::shared_ptr<ThreadsafeInput> get_input() = 0;

    virtual std::shared_ptr<const ThreadsafeInput> get_output() = 0;


};


namespace xenomai
{
class mutex
{
public:
    RT_MUTEX rt_mutex_;

    mutex()
    {
        rt_mutex_create(&rt_mutex_, NULL);
    }

    void lock()
    {
        rt_mutex_acquire(&rt_mutex_, TM_INFINITE);

    }

    void unlock()
    {
        rt_mutex_release(&rt_mutex_);
    }

};


class condition_variable
{
public:
    RT_COND rt_condition_variable_;


    condition_variable()
    {
        rt_cond_create(&rt_condition_variable_, NULL);
    }

    void wait(std::unique_lock<mutex>& lock )
    {
        rt_cond_wait(&rt_condition_variable_,
                     &lock.mutex()->rt_mutex_, TM_INFINITE);
    }

    void notify_all()
    {
        rt_cond_broadcast(&rt_condition_variable_);
    }
};
}



class ThreadsafeSingletonInterface
{
public:
#ifdef __XENO__
    typedef xenomai::mutex Mutex;
    typedef xenomai::condition_variable ConditionVariable;
#else
    typedef std::mutex Mutex;
    typedef std::condition_variable ConditionVariable;
#endif

    virtual ~ThreadsafeSingletonInterface() { }

    virtual void wait_for_update() const = 0;

    virtual void add_condition_variable(
            std::shared_ptr<ConditionVariable> condition_variable) = 0;

    virtual size_t get_modification_count() const = 0;
};


template<typename Type> class ThreadsafeSingleton: public ThreadsafeSingletonInterface
{


private:
    std::shared_ptr<Type> data_;
    mutable std::shared_ptr<Mutex> mutex_;
    mutable std::vector<std::shared_ptr<ConditionVariable>> conditions_;

    std::shared_ptr<size_t> modification_count_;

public:
    ThreadsafeSingleton()
    {
        // initialize shared pointers ------------------------------------------
        data_ = std::make_shared<Type>();
        //        condition_ = std::make_shared<ConditionVariable>();
        mutex_ = std::make_shared<Mutex>();
        conditions_.push_back(std::make_shared<ConditionVariable>());
        modification_count_ = std::make_shared<size_t>(0);
    }

    virtual const Type& get() const
    {
        std::unique_lock<Mutex> lock(*mutex_);
        return *data_;
    }

    virtual void set(const Type& datum)
    {
        // set datum in our data_ member ---------------------------------------

        std::unique_lock<Mutex> lock(*mutex_);
        *data_ = datum;
        *modification_count_ += 1;

        // notify --------------------------------------------------------------
        for(size_t i = 0; i < conditions_.size(); i++)
        {
            conditions_[i]->notify_all();
        }
    }

    virtual void wait_for_update() const
    {
        std::unique_lock<Mutex> lock(*mutex_);

        // wait until the datum with the right index is modified ---------------
        size_t initial_modification_count = *modification_count_;
        while(initial_modification_count == *modification_count_)
        {
            conditions_[0]->wait(lock);
        }

        // check that we did not miss data -------------------------------------
        if(initial_modification_count + 1 != *modification_count_)
        {
            rt_printf("something went wrong, we missed a message.\n");
            exit(-1);
        }
    }

//protected:
    virtual void add_condition_variable(
            std::shared_ptr<ConditionVariable> condition_variable)
    {
        conditions_.push_back(condition_variable);
    }

    size_t get_modification_count() const
    {
        std::unique_lock<Mutex> lock(*mutex_);
        return *modification_count_;
    }
};




template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type
tuple_for_each(std::tuple<Tp...> &, FuncT) // Unused arguments are given no names.
{ }

template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I < sizeof...(Tp), void>::type
tuple_for_each(std::tuple<Tp...>& t, FuncT f)
{
    f(std::get<I>(t));
    tuple_for_each<I + 1, FuncT, Tp...>(t, f);
}


template<typename ...Types> class ThreadsafeObjects
{
public:
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    static const std::size_t SIZE = sizeof...(Types);

#ifdef __XENO__
    typedef xenomai::mutex Mutex;
    typedef xenomai::condition_variable ConditionVariable;
#else
    typedef std::mutex Mutex;
    typedef std::condition_variable ConditionVariable;
#endif

private:

    std::shared_ptr<std::tuple<ThreadsafeSingleton<Types> ...>> data_;
    std::shared_ptr<Mutex> mutex_;
    std::shared_ptr<ConditionVariable> condition_variable_;


public:
    ThreadsafeObjects()
    {
        data_ = std::make_shared<std::tuple<ThreadsafeSingleton<Types> ...>>();
        mutex_ = std::make_shared<Mutex> ();
        condition_variable_ = std::make_shared<ConditionVariable>();


        // register condition variable everywhere ------------------------------
        tuple_for_each(*data_, [&]
                       (ThreadsafeSingletonInterface& object)->void
        {
            object.add_condition_variable(this->condition_variable_);
        });
    }


    template<int INDEX> Type<INDEX> get()
    {
        return std::get<INDEX>(*data_).get();
    }

    template<int INDEX> void set(Type<INDEX> datum)
    {
        std::get<INDEX>(*data_).set(datum);
    }

    template<int INDEX> void wait_for_update()
    {
        std::get<INDEX>(*data_).wait_for_update();
    }


    size_t get_modification_count()
    {
        size_t total_modification_count = 0;
        // register condition variable everywhere ------------------------------
        tuple_for_each(*data_, [&]
                       (const ThreadsafeSingletonInterface& object)->void
        {
            total_modification_count = total_modification_count + object.get_modification_count();
        });
        return total_modification_count;
    }

    std::array<size_t, SIZE> get_modification_counts()
    {
        std::array<size_t, SIZE> modification_counts;
        modification_counts.fill(0);
        // register condition variable everywhere ------------------------------
        size_t i = 0;
        tuple_for_each(*data_, [&]
                       (const ThreadsafeSingletonInterface& object)->void
        {
            modification_counts[i] = object.get_modification_count();
            i = i+1;
        });
        return modification_counts;
    }

    size_t wait_for_update()
    {

        std::array<size_t, SIZE> initial_modification_counts = get_modification_counts();
        size_t total_modification_count = get_modification_count();
        size_t initial_total_modification_count = total_modification_count;

        {
            std::unique_lock<Mutex> lock(*mutex_);
            while(total_modification_count == initial_total_modification_count)
            {
                condition_variable_->wait(lock);
                total_modification_count = get_modification_count();
            }
        }
        std::array<size_t, SIZE> modification_counts = get_modification_counts();


        // make sure we did not miss any data ----------------------------------
        if(total_modification_count != initial_total_modification_count + 1)
        {
            rt_printf("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );

            rt_printf("something went wrong, we missed a message.\n");
            exit(-1);
        }

        // figure out which index was modified and return it -------------------
        int modified_index = -1;
        for(size_t i = 0; i < SIZE; i++)
        {
            if(initial_modification_counts[i] + 1 == modification_counts[i])
            {
                if(modified_index != -1)
                {
                    rt_printf("something in the threadsafe object "
                              "went horribly wrong\n");
                    exit(-1);
                }

                modified_index = i;
            }
            else if(initial_modification_counts[i] !=modification_counts[i])
            {
                rt_printf("something in the threadsafe object "
                          "went horribly wrong\n");
                exit(-1);
            }
        }
        return modified_index;
    }


};







template<typename ...Types> class OldThreadsafeObject
{
public:
    template<int INDEX> using Type
    = typename std::tuple_element<INDEX, std::tuple<Types...>>::type;

    static const std::size_t SIZE = sizeof...(Types);

#ifdef __XENO__
    typedef xenomai::mutex Mutex;
    typedef xenomai::condition_variable ConditionVariable;
#else
    typedef std::mutex Mutex;
    typedef std::condition_variable ConditionVariable;
#endif

private:

    std::shared_ptr<std::tuple<Types ...> > data_;

    mutable std::shared_ptr<ConditionVariable> condition_;
    mutable std::shared_ptr<Mutex> condition_mutex_;
    std::shared_ptr<std::array<size_t, SIZE>> modification_counts_;
    std::shared_ptr<size_t> total_modification_count_;

    std::shared_ptr<std::array<Mutex, SIZE>> data_mutexes_;


public:
    OldThreadsafeObject()
    {
        // initialize shared pointers ------------------------------------------
        data_ = std::make_shared<std::tuple<Types ...> >();
        condition_ = std::make_shared<ConditionVariable>();
        condition_mutex_ = std::make_shared<Mutex>();
        modification_counts_ =
                std::make_shared<std::array<size_t, SIZE>>();
        total_modification_count_ = std::make_shared<size_t>();
        data_mutexes_ = std::make_shared<std::array<Mutex, SIZE>>();

        // initialize counts ---------------------------------------------------
        for(size_t i = 0; i < SIZE; i++)
        {
            (*modification_counts_)[i] = 0;
        }
        *total_modification_count_ = 0;
    }

    template<int INDEX=0> Type<INDEX> get() const
    {
        std::unique_lock<Mutex> lock((*data_mutexes_)[INDEX]);
        return std::get<INDEX>(*data_);
    }

    template<int INDEX=0> void set(Type<INDEX> datum)
    {
        //// \todo: we should probably add a very small wait here, to avoid the possiblity
        /// of missing messages.
        // set datum in our data_ member ---------------------------------------
        {
            std::unique_lock<Mutex> lock((*data_mutexes_)[INDEX]);
            std::get<INDEX>(*data_) = datum;
        }

        // notify --------------------------------------------------------------
        {
            std::unique_lock<Mutex> lock(*condition_mutex_);
            (*modification_counts_)[INDEX] += 1;
            *total_modification_count_ += 1;
            condition_->notify_all();
        }
    }

    void wait_for_update(unsigned index) const
    {
        std::unique_lock<Mutex> lock(*condition_mutex_);

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
            rt_printf("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );
            rt_printf("something went wrong, we missed a message.\n");
            exit(-1);
        }
    }

    template< unsigned INDEX=0> void wait_for_update() const
    {
        wait_for_update(INDEX);
    }

    size_t wait_for_update() const
    {
        std::unique_lock<Mutex> lock(*condition_mutex_);

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
            rt_printf("size: %d, \n other info: %s \n",
                      SIZE, __PRETTY_FUNCTION__ );

            rt_printf("something went wrong, we missed a message.\n");
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
                    rt_printf("something in the threadsafe object "
                              "went horribly wrong\n");
                    exit(-1);
                }

                modified_index = i;
            }
            else if(initial_modification_counts[i] !=(*modification_counts_)[i])
            {
                rt_printf("something in the threadsafe object "
                          "went horribly wrong\n");
                exit(-1);
            }
        }
        return modified_index;
    }


};

