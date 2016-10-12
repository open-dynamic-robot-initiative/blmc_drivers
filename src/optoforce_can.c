#include "optoforce_can.h"
#include <string.h>


// DEFINES
// **************************************************************************

#define OPTO_CHECK_HEADER_31(b) (\
        (b[0] == 0xAA) && (b[1] == 0x07) && (b[2] == 0x08) && (b[3] == 0x0A))

#define BYTES_TO_UINT16(b) ( (uint16_t) b[1] + ((uint16_t)b[0] << 8) )
#define BYTES_TO_SINT16(b) ( (int16_t) b[1] + ((int16_t)b[0] << 8) )


// FUNCTIONS
// **************************************************************************

void OPTO_initDataPacketBytes(OPTO_DataPacket31Bytes_t *dpb)
{
    dpb->has_high = false;
    dpb->complete = false;
}

int OPTO_decodeDataFrame(const_frame_ptr frame,
        OPTO_DataPacket31Bytes_t *data)
{
    if (frame->dlc != 8)
        return -OPTO_ERR_WRONG_DLC;
    if (data->complete)
        return -OPTO_ERR_DATA_COMPLETE;

    if (data->has_high) {
        memcpy(data->low_bytes, frame->data, frame->dlc);
        data->complete = true;
    } else {
        // check for header
        if (!OPTO_CHECK_HEADER_31(frame->data))
            return -OPTO_ERR_INVALID_HEADER;

        // TODO unnecessary copy of header
        memcpy(data->high_bytes, frame->data, frame->dlc);
        data->has_high = true;
    }

    return 0;
}

int OPTO_decodeDataPacket31(OPTO_DataPacket31Bytes_t const *bytes,
        OPTO_DataPacket31_t *pkt)
{
    if (!OPTO_checkChecksum(bytes))
        return -OPTO_ERR_WRONG_CHECKSUM;

    pkt->sample_counter = BYTES_TO_UINT16((bytes->high_bytes + 4));
    { // decode status
        uint16_t status_int = BYTES_TO_UINT16((bytes->high_bytes + 6));

        pkt->status.sensor_number = status_int & 7;
        pkt->status.multiple_sensors = (status_int >> 3) & 1;
        pkt->status.overload_tz = (status_int >> 4) & 1;
        pkt->status.overload_ty = (status_int >> 5) & 1;
        pkt->status.overload_tx = (status_int >> 6) & 1;
        pkt->status.overload_fz = (status_int >> 7) & 1;
        pkt->status.overload_fy = (status_int >> 8) & 1;
        pkt->status.overload_fx = (status_int >> 9) & 1;
        pkt->status.sensor_error = (status_int >> 10) & 7;
        pkt->status.daq_error = (status_int >> 13) & 7;
    }

    pkt->fx_counts = BYTES_TO_SINT16((bytes->low_bytes + 0));
    pkt->fy_counts = BYTES_TO_SINT16((bytes->low_bytes + 2));
    pkt->fz_counts = BYTES_TO_SINT16((bytes->low_bytes + 4));

    // TODO f*_N is not set
}

bool OPTO_checkChecksum(OPTO_DataPacket31Bytes_t const *bytes)
{
    uint16_t checksum_from_packet, checksum_computed;
    uint8_t i;

    checksum_from_packet = BYTES_TO_UINT16( (bytes->low_bytes + 6) );

    checksum_computed = 0;
    for (i = 0; i < 8; ++i) {
        checksum_computed += bytes->high_bytes[i];
    }
    for (i = 0; i < 6; ++i) {
        checksum_computed += bytes->low_bytes[i];
    }

    return checksum_from_packet == checksum_computed;
}

void OPTO_encodeConfig(uint8_t sample_freq, uint8_t filter_freq,
        uint8_t zero, uint8_t *out_nine_bytes)
{
    uint8_t* bytes = out_nine_bytes;
    uint16_t checksum;

    // header
    bytes[0] = 0xAA;
    bytes[1] = 0x00;
    bytes[2] = 0x32;
    bytes[3] = 0x03;

    bytes[4] = sample_freq;
    bytes[5] = filter_freq;
    bytes[6] = zero;

    // compute and set checksum
    checksum = 0xAA + 0x00 + 0x32 + 0x03 + sample_freq + filter_freq + zero;
    bytes[7] = checksum & 0xFF;
    bytes[8] = (checksum >> 8) & 0xFF;
}

int OPTO_sendConfig(CAN_CanHandle_t handle,
        uint8_t sample_freq, uint8_t filter_freq, uint8_t zero)
{
    CAN_CanConnection_t *can = (CAN_CanConnection_t*)handle;
    uint8_t bytes[9];
    int ret;

    OPTO_encodeConfig(sample_freq, filter_freq, zero, bytes);

    // one frame can only hold 8 bytes, so we have to split the config packet
    // into two frames
    ret = CAN_sendFrame(handle, OPTO_CAN_ID_CONFIG, bytes, 8);
    if (ret < 0)
        return ret;

    ret = CAN_sendFrame(handle, OPTO_CAN_ID_CONFIG, bytes[8], 1);
    return ret;
}
