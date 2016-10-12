/**
 * \brief Functions for decoding CAN messages from the OptoForce sensor
 *
 * \author Felix Widmaier <felix.widmaier@tuebingen.mpg.de>
 */
#ifndef OPTOFORCE_CAN_H_
#define OPTOFORCE_CAN_H_

// INCLUDES
// **************************************************************************
#include <stdbool.h>
#include "can.h"


// DEFINES
// **************************************************************************

// Arbitration IDs of the different message types
#define OPTO_CAN_ID_SENSOR_DATA 0x100
#define OPTO_CAN_ID_CONFIG 0x101


// DAQ errors
#define OPTO_DAQ_ERR_NONE 0
#define OPTO_DAQ_ERR_DAQ 1
#define OPTO_DAQ_ERR_COMMUNICATION 2

// Sensor errors
#define OPTO_SENSOR_ERR_NONE 0
#define OPTO_SENSOR_ERR_NOT_DETECTED 1
#define OPTO_SENSOR_ERR_FAILURE 2
#define OPTO_SENSOR_ERR_TEMPERATURE 4

#define OPTO_CONFIG_SAMPLE_FREQ_NONE 0
#define OPTO_CONFIG_SAMPLE_FREQ_1000 1
#define OPTO_CONFIG_SAMPLE_FREQ_333  3
#define OPTO_CONFIG_SAMPLE_FREQ_100 10 // default
#define OPTO_CONFIG_SAMPLE_FREQ_30  33 // TODO: really??
#define OPTO_CONFIG_SAMPLE_FREQ_10 100

#define OPTO_CONFIG_FILTER_FREQ_NONE 0
#define OPTO_CONFIG_FILTER_FREQ_500  1
#define OPTO_CONFIG_FILTER_FREQ_150  2
#define OPTO_CONFIG_FILTER_FREQ_50   3
#define OPTO_CONFIG_FILTER_FREQ_15   4 // default
#define OPTO_CONFIG_FILTER_FREQ_5    5
#define OPTO_CONFIG_FILTER_FREQ_1_5  6 // 1.5 Hz

#define OPTO_CONFIG_SET_ZERO 255
#define OPTO_CONFIG_UNZERO 0

// header of config packets
#define OPTO_PACKET_HEADER_CONFIG 0xAA003203


// error codes
#define OPTO_ERR_WRONG_DLC 1
#define OPTO_ERR_DATA_COMPLETE 2
#define OPTO_ERR_INVALID_HEADER 3
#define OPTO_ERR_WRONG_CHECKSUM 4


// TYPEDEFS
// **************************************************************************

typedef CAN_Frame_t const * const const_frame_ptr;


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

typedef OPTO_DataPacket31_t OPTO_SensorData_t;


typedef struct _OPTO_DataPacket31Bytes_t_
{
    bool has_high;
    bool complete;
    uint8_t high_bytes[8];
    uint8_t low_bytes[8];
} OPTO_DataPacket31Bytes_t;


typedef struct _OPTO_ConfigPacket_t_
{
    uint8_t sample_freq;
    uint8_t filter_freq;
    uint8_t zero;
} OPTO_ConfigPacket_t;


// FUNCTIONS
// **************************************************************************


void OPTO_initDataPacketBytes(OPTO_DataPacket31Bytes_t *dpb);


// FIXME rename
int OPTO_decodeDataFrame(const_frame_ptr frame,
        OPTO_DataPacket31Bytes_t *data);


int OPTO_decodeDataPacket31(OPTO_DataPacket31Bytes_t const *bytes,
        OPTO_DataPacket31_t *pkt);


bool OPTO_checkChecksum(OPTO_DataPacket31Bytes_t const *bytes);


void OPTO_encodeConfig(uint8_t sample_freq, uint8_t filter_freq,
        uint8_t zero, uint8_t *out_nine_bytes);


int OPTO_sendConfig(CAN_CanHandle_t handle,
        uint8_t sample_freq, uint8_t filter_freq, uint8_t zero);

#endif  // OPTOFORCE_CAN_H_
