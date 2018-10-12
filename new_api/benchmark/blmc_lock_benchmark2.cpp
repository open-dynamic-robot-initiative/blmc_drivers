#include <eigen3/Eigen/Core>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sched.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include <devices/analog_sensor.hpp>

#include <utils/timer.hpp>

typedef ThreadsafeTimeseries<double> ScalarTimeseries;



/**
 * Computes the time as target = a - b;
 */
void timespec_sub(struct timespec *target, struct timespec *a, struct timespec *b)
{
    if (b->tv_nsec > a->tv_nsec) {
        target->tv_nsec = (a->tv_nsec - b->tv_nsec) + 1000000000;
        target->tv_sec = a->tv_sec - b->tv_sec - 1;
    } else {
        target->tv_nsec = a->tv_nsec - b->tv_nsec;
        target->tv_sec = a->tv_sec - b->tv_sec;
    }
}

ThreadsafeTimeseries<double> series(1000);

static void
#ifndef __XENO__
*
#endif
thread_body_locking(void* index)
{
    int print_counter = 1000000;

//    cpu_set_t cpuset;
//    CPU_ZERO(&cpuset);
//    CPU_SET(*((int*)(index)) + 1, &cpuset);
//    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    // if (rc != 0) {
      // printf("Error calling pthread_setaffinity_np: %d\n", rc);
    // }

    osi::realtime_printf("Hello world from thread id=%d, cpu=%d\n", syscall(SYS_gettid), sched_getcpu());

    while(true)
    {
        Timer<1> timer;

        double d = 0.;
        size_t cpu_index = sched_getcpu();
        bool cpu_switch = false;
        for(size_t i = 0; i < print_counter; i++)
        {
            if(sched_getcpu() != cpu_index)
            {
                cpu_switch = true;
            }

            timer.end_and_start_interval();

//            osi::sleep_ms(0.000000001);

            series.append(d);
            d = series.newest_element();
            d += 1.;
        }
        osi::realtime_printf("%d store and reads from thread %d, cpu=%d, cpu_switch=%d\n",
                             print_counter,
                             *(int*)(index),
                             sched_getcpu(),
                             cpu_switch);
        timer.print_status();
    }
}

static void
#ifndef __XENO__
*
#endif
thread_body_math(void* instance_pointer)
{
  struct timespec now;
  struct timespec prev;
  struct timespec elapsed;

  double d = 0.;
  long long i = 0;
  int print_counter = 1;

  Eigen::MatrixXd m = Eigen::MatrixXd::Random(1024, 1024);

  osi::realtime_printf("Start benchmarking.\n");

  clock_gettime(CLOCK_REALTIME, &now);

  while (true) {
    m = m * m;
    i ++;

    if (i % print_counter == 0) {
      clock_gettime(CLOCK_REALTIME, &now);
      timespec_sub(&elapsed, &now, &prev);

      float elapsed_ms = (float)(elapsed.tv_sec)*1000 + (elapsed.tv_nsec/1e6);
      osi::realtime_printf(
        "Duration %0.6f ms for %d matrix multiplies. m(42, 42)=%0.3f\n",
        elapsed_ms, print_counter, m(512, 512));

      prev = now;
    }
  }
}


int main(int argc, char **argv)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);

    osi::initialize_realtime_printing();

    osi::realtime_printf("argc= %d\n", argc);

    std::vector<int> indices(4);
    for(size_t i = 0; i < indices.size(); i++)
        indices[i] = i;

    if (argc == 1) {
      for (int i = 0; i < 4; i++) {
        osi::start_thread(&thread_body_locking, &indices[i]);
      }
    } else if (argc == 2) {
      osi::start_thread(&thread_body_math, NULL);
    }

    while(true)
    {
        Timer<>::sleep_ms(10);
    }
}
