#pragma once

#include <utils/os_interface.hpp>

template<typename DataType> class StampedData
{
    /// public interface =======================================================
public:
    // getters -----------------------------------------------------------------
    const DataType& get_data() const
    {
        return data_;
    }
    const size_t& get_id() const
    {
        return id_;
    }
    const double& get_time_stamp() const
    {
        return time_stamp_;
    }

    // setter ------------------------------------------------------------------
    void set_data(const DataType& data)
    {
        data_ = data;
    }

    template<typename Type>
    void set_header_equal(const StampedData<Type>& other_stamped_data)
    {
        id_ = other_stamped_data.get_id();
        time_stamp_ = other_stamped_data.get_time_stamp();
    }

    // constructors ------------------------------------------------------------
    StampedData():
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    StampedData(const DataType& data):
        data_(data),
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    StampedData(const DataType& data,
                const size_t& id,
                const double& time_stamp):
        data_(data),
        id_(id),
        time_stamp_(time_stamp) {  }

    template<typename Type>
    StampedData(const StampedData<Type>& other_stamped_data,
                const DataType& data)
    {
        data_ = data;
        set_header_equal(other_stamped_data);
    }

    void print_header() const
    {
        osi::printf("id: %d, time_stamp: %f\n", id_, time_stamp_);
    }

    /// private data ===========================================================
private:
    DataType data_;
    size_t id_;
    double time_stamp_;
};
