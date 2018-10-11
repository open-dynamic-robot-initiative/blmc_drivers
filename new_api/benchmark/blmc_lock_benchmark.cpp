#include <eigen3/Eigen/Core>
#include <time.h>

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


    struct timespec now;
    struct timespec prev;
    struct timespec elapsed;

    double d = 0.;
    long long i = 0;
    int print_counter = 100000;

    clock_gettime(CLOCK_REALTIME, &now);
    prev = now;
    while (true) {

        series.append(d);
        d = series.newest_element();
        d += 1.;
        i ++;


        if (i % print_counter == 0)
        {
            //        timer.end_and_start_interval();
            clock_gettime(CLOCK_REALTIME, &now);
            timespec_sub(&elapsed, &now, &prev);

            float elapsed_ms = (float)(elapsed.tv_sec)*1000 + (elapsed.tv_nsec/1e6);
            osi::realtime_printf("Duration %0.6f ms for %d store and reads from thread %d\n", elapsed_ms, print_counter, *(int*)(index));

            prev = now;
        }
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

  Eigen::MatrixXd m = Eigen::MatrixXd::Random(2048, 2048);

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
    osi::initialize_realtime_printing();

    osi::realtime_printf("argc= %d\n", argc);

    std::vector<int> indices(4);
    for(size_t i = 0; i < indices.size(); i++)
        indices[i] = i;

    if (argc == 1) {
        for(size_t i = 0; i < indices.size(); i++)
            osi::start_thread(&thread_body_locking, &indices[i]);
//      osi::start_thread(&thread_body_locking, NULL);
//      osi::start_thread(&thread_body_locking, NULL);
//      osi::start_thread(&thread_body_locking, NULL);
    } else if (argc == 2) {
      osi::start_thread(&thread_body_math, NULL);
    }


    while(true)
    {
        Timer<>::sleep_ms(10);
    }
}
