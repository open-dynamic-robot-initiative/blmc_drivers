#pragma once

#include <blmc_drivers/utils/os_interface.hpp>

/**
 * @brief This class define a data which has a header. So it is a stamped data.
 * 
 * @tparam DataType could be anything, this class will define a common header
 * for all data type.
 */
template<typename DataType> class StampedData
{
public:

    /**
     * @brief Construct a new StampedData object
     */
    StampedData():
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    /**
     * @brief Construct a new StampedData object
     * 
     * @param data value.
     */
    StampedData(const DataType& data):
        data_(data),
        id_(std::numeric_limits<size_t>::quiet_NaN()),
        time_stamp_(std::numeric_limits<double>::quiet_NaN()) {  }

    /**
     * @brief Construct a new StampedData object
     * 
     * @param data value
     * @param id is the identification number of the header.
     * @param time_stamp is the data time stamp
     */
    StampedData(const DataType& data,
                const size_t& id,
                const double& time_stamp):
        data_(data),
        id_(id),
        time_stamp_(time_stamp) {  }

    /**
     * @brief Construct a new StampedData object
     * 
     * @tparam Type is the type of data to stamp
     * @param other_stamped_data another data stamped
     * @param data the value to copy.
     */
    template<typename Type>
    StampedData(const StampedData<Type>& other_stamped_data,
                const DataType& data)
    {
        data_ = data;
        set_header_equal(other_stamped_data);
    }

    /**
     * @brief set_header_equal copy the header from another data.
     * 
     * @tparam Type 
     * @param other_stamped_data 
     */
    template<typename Type>
    void set_header_equal(const StampedData<Type>& other_stamped_data)
    {
        id_ = other_stamped_data.get_id();
        time_stamp_ = other_stamped_data.get_time_stamp();
    }

    /**
     * @brief Display this object in a terminal
     */
    void print_header() const
    {
        rt_printf("id: %d, time_stamp: %f\n", id_, time_stamp_);
    }

    /**
     * Getters
     */

    /**
     * @brief Get the data_ object
     * 
     * @return const DataType& 
     */
    const DataType& get_data() const
    {
        return data_;
    }

    /**
     * @brief Get the id_ object
     * 
     * @return const size_t& 
     */
    const size_t& get_id() const
    {
        return id_;
    }

    /**
     * @brief Get the time_stamp_ object
     * 
     * @return const double& 
     */
    const double& get_time_stamp() const
    {
        return time_stamp_;
    }

    /**
     * Setters
     */

    /**
     * @brief Set the data_ object
     * 
     * @param data 
     */
    void set_data(const DataType& data)
    {
        data_ = data;
    }

    /// private data ===========================================================
private:
    /**
     * @brief the data to stamp.
     */
    DataType data_;

    /**
     * @brief the id of the header.
     */
    size_t id_;

    /**
     * @brief the time stamp.
     */
    double time_stamp_;
};
