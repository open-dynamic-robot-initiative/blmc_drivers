#pragma once



#ifdef __XENO__

#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/cond.h>

#include <rtdk.h>
#include <rtdm/rtcan.h>

#else
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <limits.h>


// Define typedefs to make code compatible with Xenoami code.
typedef struct can_frame can_frame_t;
typedef canid_t can_id_t;
typedef uint64_t 	nanosecs_abs_t;

#define rt_fprintf fprintf
#define rt_printf printf



#define rt_dev_socket socket
#define rt_dev_ioctl ioctl
#define rt_dev_close close
#define rt_dev_setsockopt setsockopt
#define rt_dev_bind bind
#define rt_dev_recvmsg recvmsg
#define rt_dev_sendto sendto
#endif

#include <mutex>
#include <condition_variable>

#include <sys/mman.h>


namespace osi
{



#ifdef __XENO__
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


    typedef xenomai::mutex Mutex;
    typedef xenomai::condition_variable ConditionVariable;
#else
    typedef std::mutex Mutex;
    typedef std::condition_variable ConditionVariable;
#endif



//template<typename ...Ts>
// inline   void realtime_printf(Ts... args)
//{
//    rt_printf(args...);

//}

inline void send_to_can_device(int fd, const void *buf, size_t len,
                       int flags, const struct sockaddr *to,
                       socklen_t tolen)
{
    int ret = rt_dev_sendto(fd, buf, len, flags, to, tolen);

    if (ret < 0)
    {
        rt_printf("something went wrong with "
                  "sending CAN frame, error code: %d, errno=%d\n", ret, errno);
        exit(-1);
    }
}

inline void close_can_device(int socket)
{
    int ret = rt_dev_close(socket);
    if (ret)
    {
        rt_fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
        exit(-1);
    }
}

inline void receive_message_from_can_device(int fd, struct msghdr *msg, int flags)
{
    int ret = rt_dev_recvmsg(fd, msg, flags);
    if (ret < 0)
    {
        rt_printf("something went wrong with receiving "
                  "CAN frame, error code: %d, errno=%d\n", ret, errno);
        exit(-1);
    }

}
#ifdef __XENO__
#define THREAD_FUNCTION_RETURN_TYPE void
#else
#define THREAD_FUNCTION_RETURN_TYPE void*
#endif

inline void start_thread(THREAD_FUNCTION_RETURN_TYPE (*function)(void *cookie),
                         void *argument=NULL)
{
#ifdef __XENO__

    // TODO: not sure if this is the right place for this
    mlockall(MCL_CURRENT | MCL_FUTURE);
    //        signal(SIGTERM, cleanup_and_exit);
    //        signal(SIGINT, cleanup_and_exit);
    //        signal(SIGDEBUG, action_upon_switch);
    rt_print_auto_init(1);

    RT_TASK rt_task;
    int priority = 10;

    int return_task_create = rt_task_create(&rt_task, NULL, 0, priority,  T_JOINABLE | T_FPU);
    if (return_task_create) {
        rt_fprintf(stderr, "controller: %s\n", strerror(-return_task_create));
        exit(-1);
    }
    rt_task_start(&rt_task, function, argument);

#else
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    int ret;

    ret = pthread_attr_init(&attr);
    if (ret) {
        printf("init pthread attributes failed\n");
    }

    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, function, argument);
    if (ret) {
        printf("create pthread failed. Ret=%d\n", ret);
        if (ret == 1) {
            printf("NOTE: This program must be executed as root to get the "
                "required realtime permissions.\n");
        }
    }
#endif
}


inline void initialize_realtime_printing()
{
#ifdef __XENO__
    rt_print_auto_init(1);
#endif
}


inline void sleep_ms(const double& sleep_time_ms)
{
#ifdef __XENO__
    rt_task_sleep(int(sleep_time_ms * 1000000.));
#else
    usleep(sleep_time_ms * 1000.);
#endif
}

inline double get_current_time_ms()
{
#ifdef __XENO__
    return double(rt_timer_read()) / 1000000.;
#else
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    double current_time_ms = (double)(now.tv_sec*1e3) + (now.tv_nsec/1e6);

    return current_time_ms;



#endif
}




inline void make_this_thread_realtime()
{
#ifdef __XENO__

    rt_task_shadow(NULL, NULL, 0, 0);
#endif
}
}
