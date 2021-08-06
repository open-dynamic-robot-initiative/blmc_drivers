/**
 * @file os_interface.hpp
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief
 * @version 0.1
 * @date 2018-11-27
 *
 * @copyright Copyright (c) 2018
 *
 */

#pragma once

/**
 * xeno specific include
 */
#ifdef __XENO__

#include <native/cond.h>
#include <native/mutex.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <rtdm/rtcan.h>

/**
 * Ubuntu posix rt_prempt based include
 */
#else

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <limits.h>

#include <real_time_tools/timer.hpp>

#include <iostream>

/**
 * Define typedefs to make code compatible with Xenomai code.
 */

/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
typedef struct can_frame can_frame_t;
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
typedef canid_t can_id_t;
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
typedef uint64_t nanosecs_abs_t;

/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_fprintf fprintf
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_printf printf

/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_socket socket
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_ioctl ioctl
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_close close
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_setsockopt setsockopt
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_bind bind
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_recvmsg recvmsg
/**
 * @brief Create a common type_def to wrap xenomai and posix.
 */
#define rt_dev_sendto sendto

/**
 * Common include
 */
#endif

#include <condition_variable>
#include <mutex>
#include <sstream>

#include <sys/mman.h>

/**
 * @brief osi stands for Operating System Interface.
 * \todo This workspace should be replaced eventually by the real_time_tools
 * package.
 */
namespace osi
{
#ifdef __XENO__

/**
 * Namespace to wrap the xenomai specific implementation xenomai
 */
namespace xenomai
{
/**
 * @brief The mutex class is a specific implementation of the mutex class
 * for xenomai
 */
class mutex
{
public:
    /**
     * @brief Create a xenomai mutex object.
     */
    RT_MUTEX rt_mutex_;

    /**
     * @brief Construct a new mutex object
     */
    mutex()
    {
        rt_mutex_create(&rt_mutex_, NULL);
    }

    /**
     * @brief lock the mutex.
     */
    void lock()
    {
        rt_mutex_acquire(&rt_mutex_, TM_INFINITE);
    }

    /**
     * @brief Unlock the mutex.
     */
    void unlock()
    {
        rt_mutex_release(&rt_mutex_);
    }
};

/**
 * @brief Implementation of a condition variable specific to xenomai
 */
class condition_variable
{
public:
    /**
     * @brief Create the xenomai condition variable object
     */
    RT_COND rt_condition_variable_;

    /**
     * @brief Construct a new condition_variable object
     */
    condition_variable()
    {
        rt_cond_create(&rt_condition_variable_, NULL);
    }

    /**
     * @brief Put the condition variable to wait mode.
     *
     * @param lock is the mutex to be used for locking the scope.
     */
    void wait(std::unique_lock<mutex> &lock)
    {
        rt_cond_wait(
            &rt_condition_variable_, &lock.mutex()->rt_mutex_, TM_INFINITE);
    }

    /**
     * @brief Notify all condition variable owning the same mutex.
     */
    void notify_all()
    {
        rt_cond_broadcast(&rt_condition_variable_);
    }
};
}  // namespace xenomai
/**
 * @brief Wrapper around the xenomai specific Mutex implementation.
 */
typedef xenomai::mutex Mutex;

/**
 * @brief Wrapper around the xenomai specific ConditionVariable
 * implementation
 */
typedef xenomai::condition_variable ConditionVariable;

#else
/**
 * @brief Wrapper around the posix specific Mutex implementation.
 */
typedef std::mutex Mutex;

/**
 * @brief Wrapper around the posix specific ConditionVariable
 * implementation
 */
typedef std::condition_variable ConditionVariable;
#endif

/**
 * @brief Use the osi workspace API to communicate with the can bus.
 * /todo Manuel can you describe the argument of this function?
 * @param fd
 * @param buf
 * @param len
 * @param flags
 * @param to
 * @param tolen
 */
inline void send_to_can_device(int fd,
                               const void *buf,
                               size_t len,
                               int flags,
                               const struct sockaddr *to,
                               socklen_t tolen)
{
    // int ret = rt_dev_sendto(fd, buf, len, flags, to, tolen);

    // if (ret < 0)
    // {
    //     std::ostringstream oss;
    //     oss << "something went wrong with sending "
    //         << "CAN frame, error code: "
    //         << ret << ", errno=" << errno << std::endl;
    //     throw std::runtime_error(oss.str());
    // }

    for (size_t i = 0; true; i++)
    {
        int ret = rt_dev_sendto(fd, buf, len, flags, to, tolen);
        if (ret >= 0)
        {
            if (i > 0)
            {
                std::cout << " Managed to send after " << i << " attempts."
                          << std::endl;
            }
            return;
        }

        if (i == 0)
        {
            std::cout << "WARNING: Something went wrong with sending "
                      << "CAN frame, error code: " << ret
                      << ", errno: " << errno << ". Possibly you have "
                      << "been attempting to send at a rate which is too "
                      << "high. We keep trying" << std::flush;
        }

        real_time_tools::Timer::sleep_ms(0.1);
    }
}

/**
 * @brief This function is closing a socket on the Can device. It is os
 *  independent.
 *
 * @param socket
 */
inline void close_can_device(int socket)
{
    int ret = rt_dev_close(socket);
    if (ret)
    {
        rt_fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
        exit(-1);
    }
}

/**
 * @brief Poll? a message from the CAN device.
 * \todo Manuel can you confrim this? And precise the arguments of the function?
 *
 * @param fd
 * @param msg
 * @param flags
 */
inline void receive_message_from_can_device(int fd,
                                            struct msghdr *msg,
                                            int flags)
{
    int ret = rt_dev_recvmsg(fd, msg, flags);
    if (ret < 0)
    {
        std::ostringstream oss;
        oss << "something went wrong with receiving "
            << "CAN frame, error code: " << ret << ", errno=" << errno
            << std::endl;
        throw std::runtime_error(oss.str());
    }
}

/**
 * @brief This function is needed in xenomai to initialize the real time console
 * display of text.
 */
inline void initialize_realtime_printing()
{
#ifdef __XENO__
    rt_print_auto_init(1);
#endif
}

/**
 * @brief This function uses eather the xenomai API or the posix one.
 *
 * @param sleep_time_ms is the sleeping time in milli seconds.
 */
inline void sleep_ms(const double &sleep_time_ms)
{
#ifdef __XENO__
    rt_task_sleep(int(sleep_time_ms * 1000000.));
#else
    usleep(sleep_time_ms * 1000.);  // nano_sleep
#endif
}

/**
 * @brief Get the current time in millisecond.
 * \todo remove as the one form the Timer class is much better embeded.
 *
 * @return double which is the time in milli second
 */
inline double get_current_time_ms()
{
#ifdef __XENO__
    return double(rt_timer_read()) / 1000000.;
#else
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    double current_time_ms = (double)(now.tv_sec * 1e3) + (now.tv_nsec / 1e6);

    return current_time_ms;
#endif
}

/**
 * @brief This methd is requiered in xenomai to create a real time thread.
 */
inline void make_this_thread_realtime()
{
#ifdef __XENO__

    rt_task_shadow(NULL, NULL, 0, 0);
#endif
}

}  // namespace osi
