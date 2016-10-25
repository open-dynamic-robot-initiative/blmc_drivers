/**
 * \file
 * \brief Functions for decoding CAN messages from the OptoForce sensor
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 *
 * \defgroup OptoForceAPI OptoForce API
 * \brief API for communication with the OptoForce sensor via CAN.
 *
 * \{
 */
#ifndef OPTOFORCE_CAN_H_
#define OPTOFORCE_CAN_H_

// INCLUDES
// **************************************************************************
#include <stdbool.h>
#include <blmc_can/can.h>


#ifdef __cplusplus
extern "C"{
#endif


// DEFINES
// **************************************************************************

//! \name Arbitration IDs
//! \brief Arbitration IDs of the different message types.
//! \{
#define OPTO_CAN_ID_SENSOR_DATA 0x100
#define OPTO_CAN_ID_CONFIG 0x101
//! \}


//! \name DAQ errors
//! \{
#define OPTO_DAQ_ERR_NONE 0
#define OPTO_DAQ_ERR_DAQ 1
#define OPTO_DAQ_ERR_COMMUNICATION 2
//! \}

//! \name Sensor errors
//! \{
#define OPTO_SENSOR_ERR_NONE 0
#define OPTO_SENSOR_ERR_NOT_DETECTED 1
#define OPTO_SENSOR_ERR_FAILURE 2
#define OPTO_SENSOR_ERR_TEMPERATURE 4
//! \}


//! \name Sample Frequency Options
//! \brief Possible values for configuration of the sample frequency.
//!
//! Default is 100 Hz.
//!
//! \{

//! \brief Disable sampling
#define OPTO_CONFIG_SAMPLE_FREQ_NONE 0
//! \brief Sample with 1 kHz
#define OPTO_CONFIG_SAMPLE_FREQ_1000 1
//! \brief Sample with 333 Hz
#define OPTO_CONFIG_SAMPLE_FREQ_333  3
//! \brief Sample with 100 Hz. This is the default.
#define OPTO_CONFIG_SAMPLE_FREQ_100 10 // default
//! \brief Sample with 33 Hz
#define OPTO_CONFIG_SAMPLE_FREQ_30  33 // TODO: really??
//! \brief Sample with 10 Hz
#define OPTO_CONFIG_SAMPLE_FREQ_10 100
//! \}


//! \name Filter Frequency Options
//! \brief Possible values for configuration of the filter frequency.
//!
//! Default is 15 Hz.
//!
//! \{

//! \brief Disable filter
#define OPTO_CONFIG_FILTER_FREQ_NONE 0
//! \brief Filter with 500 Hz
#define OPTO_CONFIG_FILTER_FREQ_500  1
//! \brief Filter with 150 Hz
#define OPTO_CONFIG_FILTER_FREQ_150  2
//! \brief Filter with 50 Hz
#define OPTO_CONFIG_FILTER_FREQ_50   3
//! \brief Filter with 15 Hz
#define OPTO_CONFIG_FILTER_FREQ_15   4 // default
//! \brief Filter with 5 Hz
#define OPTO_CONFIG_FILTER_FREQ_5    5
//! \brief Filter with 1.5 Hz
#define OPTO_CONFIG_FILTER_FREQ_1_5  6 // 1.5 Hz
//! \}


//! \name Zero Options
//! \brief Possible values for (un-)zeroing the sensor.
//! \{

//! \brief Zero sensor based on the current offset.
#define OPTO_CONFIG_SET_ZERO 255
//! \brief Remove zero offsets.
#define OPTO_CONFIG_UNZERO 0
//! \}


//! \name Function Error Return Codes
//! \brief Codes that are used as return value by some of the functions to
//!        indicate errors.
//! \{

//! \brief Given CAN frame has wrong number of data bytes.
#define OPTO_ERR_WRONG_DLC 1
//! \brief Trying to add bytes to an OptoForce packet that is already complete.
#define OPTO_ERR_DATA_COMPLETE 2
//! \brief Header of the OptoForce packet is wrong.
#define OPTO_ERR_INVALID_HEADER 3
//! \brief Checksum of the OptoForce packet is wrong.
#define OPTO_ERR_WRONG_CHECKSUM 4
//! \brief Trying to process an incomplete packet.
#define OPTO_ERR_INCOMPLETE_PACKET 5
//! \}

//! \name Function Return Codes
//! \brief Codes that are used as return value by some of the funcitons to
//!        indicate an certain result that is not an error.
//! \{

//! \brief ???
#define OPTO_RET_DATA_COMPLETE 0
//! \brief The frame does not come from the sensor and is ignored.
#define OPTO_RET_NO_OPTO_FRAME 1
//! \brief The frame was processed but the OptoForce packet is not yet
//!        complete (i.e. waiting for the next frame).
#define OPTO_RET_DATA_INCOMPLETE 2
//! \}


// TYPEDEFS
// **************************************************************************

typedef CAN_Frame_t const * const const_frame_ptr;


//! \brief OptoForce Sensor Status
typedef struct _OPTO_Status_t_
{
    uint16_t sensor_number:3;     // 0-2
    uint16_t multiple_sensors:1;  // 3
    uint16_t overload_tz:1;       // 4
    uint16_t overload_ty:1;       // 5
    uint16_t overload_tx:1;       // 6
    uint16_t overload_fz:1;       // 7
    uint16_t overload_fy:1;       // 8
    uint16_t overload_fx:1;       // 9
    uint16_t sensor_error:3;      // 10-12
    uint16_t daq_error: 3;        // 13-15
} OPTO_Status_t;


//! \brief OptoForce Sensor Data Packet Type 31
typedef struct _OPTO_DataPacket31_t_
{
    uint16_t sample_counter;
    OPTO_Status_t status;
    //! Force on x-axis in raw counts
    int16_t fx_counts;
    //! Force on y-axis in raw counts
    int16_t fy_counts;
    //! Force on z-axis in raw counts
    int16_t fz_counts;
    //! Force on x-axis in Newton
    float fx_N;
    //! Force on y-axis in Newton
    float fy_N;
    //! Force on z-axis in Newton
    float fz_N;

} OPTO_DataPacket31_t;

// give it a nicer name
typedef OPTO_DataPacket31_t OPTO_SensorData_t;


//! \brief OptoForce Sensor Data Packet Bytes Type 31
typedef struct _OPTO_DataPacket31Bytes_t_
{
    bool has_high;  //!< If true, high_bytes is set
    bool complete;  //!< If true, both high_bytes and low_bytes is set
    uint8_t high_bytes[8];  //!< the higher 8 bytes of the packet
    uint8_t low_bytes[8];   //!< the lower 8 bytes of the packet
} OPTO_DataPacket31Bytes_t;


// FUNCTIONS
// **************************************************************************


//! \brief Initialize data packet bytes structure.
void OPTO_initDataPacketBytes(OPTO_DataPacket31Bytes_t *dpb);


// FIXME rename
//! \brief Store the data of the frame in the packet bytes structure.
int OPTO_decodeDataFrame(const_frame_ptr frame,
        OPTO_DataPacket31Bytes_t *bytes);


//! \brief Decode the given data packet bytes
//! \param bytes The data packet bytes.
//! \param pkt [out] The decoded data is stored here (only if function returns
//!                  without error).
//! \returns 0 if everything is fine, negative error code otherwise.
int OPTO_decodeDataPacket31(OPTO_DataPacket31Bytes_t const *bytes,
        OPTO_DataPacket31_t *pkt);


//! \brief Check if the cecksum of the given packet is correct
//! \param bytes The data packet bytes.
//! \returns True if checksum is correct, false if not.
bool OPTO_checkChecksum(OPTO_DataPacket31Bytes_t const *bytes);


//! \brief Encode a configuration packet.
//! \param sample_freq See OPTO_sendConfig.
//! \param filter_freq See OPTO_sendConfig.
//! \param zero See OPTO_sendConfig.
//! \param out_nine_bytes [out] The nine bytes of config packet are written
//!                       to this array.
void OPTO_encodeConfig(uint8_t sample_freq, uint8_t filter_freq,
        uint8_t zero, uint8_t *out_nine_bytes);


//! \brief Send a configuration packet to the sensor.
//! \param sample_freq Sample frequency of the sensor. Use the
//!                    OPTO_CONFIG_SAMPLE_FREQ_* defines.
//! \param filter_freq Frequency of the built-in filter of the sensor. Use the
//!                    OPTO_CONFIG_FILTER_FREQ_* defines.
//! \param zero If set to OPTO_CONFIG_SET_ZERO, the sensor is zeroed, if set to
//!             OPTO_CONFIG_UNZERO pervious zero offsets are removed.
//! \returns Return value of CAN_sendFrame.
int OPTO_sendConfig(CAN_CanHandle_t handle,
        uint8_t sample_freq, uint8_t filter_freq, uint8_t zero);


//! \brief Process a CAN frame.
//!
//! Receiving data from the sensor via CAN is a bit tricky because the 16 byte
//! data packet is split into two CAN frames.  This means the first frame has
//! to be stored and can only be processed when the second one arives.  Only in
//! the latter case will the decoded data stored to the output parameter
//! `data`.  To see when this happens, check the return value of the function.
//! There are three possible cases, depending on the content of frame:
//!
//! 1) If the frame is not sent by the sensor, nothing happens.
//!    OPTO_RET_NO_OPTO_FRAME is returned.
//!
//! 2) If the frame is sent by the sensor and the first part of a data frame is
//!    already stored, the current frame is assumed to be the second part.  The
//!    packet is decoded and, if successful, the content is stored to `data`.
//!    The return value of OPTO_decodeDataPacket31 is returned.
//!
//! 3) If the frame is sent by the sensor and no previous data is stored:
//!    3.1) If the frame begins with the data packet header, it is stored and
//!         OPTO_RET_DATA_INCOMPLETE is returned.
//!    3.2) If the frame does not begin with the data packet header,
//!         -OPTO_ERR_INVALID_HEADER is returned.
//!
//! Note that `data` is only filled if the function returns with 0. In all
//! other cases, data is left unchanged.
//!
//! \param frame The CAN frame that is to be processed.
//! \param data If return value is 0, this contains the values of the processed
//!             data packet, otherwise it is unchanged.
//! \return See above.
int OPTO_processCanFrame(const_frame_ptr frame, OPTO_DataPacket31_t *data);

#ifdef __cplusplus
}
#endif

/** \} */ //end group OptoForceAPI

#endif  // OPTOFORCE_CAN_H_
