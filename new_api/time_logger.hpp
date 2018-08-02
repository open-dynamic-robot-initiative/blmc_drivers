#pragma once

#include <native/timer.h>
#include <rtdk.h>

#include <limits>
#include <eigen3/Eigen/Core>




template<unsigned LENGTH> class TimeLogger
{
private:
    double min_interval_;
    double max_interval_;
    double total_interval_;

    Eigen::Matrix<double, LENGTH, 1> intervals_;

    long unsigned count_;
    long unsigned print_period_;

    double interval_start_;

    std::string name_;

public:
    TimeLogger(std::string name = "time logger", long unsigned print_period = 0)
    {
        min_interval_ = std::numeric_limits<double>::max();
        max_interval_ = std::numeric_limits<double>::min();
        total_interval_;
        intervals_.setZero();

        count_ = 0;
        print_period_ = print_period;

        interval_start_ = std::numeric_limits<double>::quiet_NaN();

        name_ = name;
    }

    // timing functionality ----------------------------------------------------
    void start_interval(double interval_start)
    {
        interval_start_ = interval_start;
    }
    void start_interval()
    {
        start_interval(current_time_ms());
    }

    void end_interval(double interval_end)
    {
        if(!std::isnan(interval_start_))
        {
            add_interval(interval_end - interval_start_);
        }

        if(print_period_ && count_ % print_period_ == 0)
        {
            print_status();
        }
    }
    void end_interval()
    {
        end_interval(current_time_ms());
    }

    void end_and_start_interval()
    {
        double time = current_time_ms();
        end_interval(time);
        start_interval(time);
    }

    // getters -----------------------------------------------------------------
    double max_interval() const
    {
        if(count_ < LENGTH)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return max_interval_;
    }
    double min_interval() const
    {
        if(count_ < LENGTH)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return min_interval_;
    }
    double avg_interval() const
    {
        if(count_ < LENGTH)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return total_interval_ / double(LENGTH);
    }
    long unsigned count() const
    {
        return count_;
    }

    // operating system specific functions -------------------------------------
    void print_status() const
    {
        rt_printf("%s --------------------------------\n", name_.c_str());
        rt_printf("count: %d\n min_interval: %f\n max_interval: %f\n avg_interval: %f\n",
                  count_, min_interval(), max_interval(), avg_interval());
        rt_printf("--------------------------------------------\n");

    }

    static double current_time_ms()
    {
        return double(rt_timer_read()) / 1000000.;
    }

private:
    void add_interval(double interval)
    {
        min_interval_ = interval < min_interval_ ? interval : min_interval_;
        max_interval_ = interval > max_interval_ ? interval : max_interval_;

        long unsigned index = count_ % LENGTH;
        total_interval_ += interval - intervals_[index];
        intervals_[index] = interval;

        count_++;
    }
};
