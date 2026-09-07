#ifndef ANELLO_PROTOCOL_TYPES_H
#define ANELLO_PROTOCOL_TYPES_H

#include <cstdint>
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "ANELLO packed binary decoding requires a little-endian host"
#endif
#include "bit_tools.h"

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN 1200
#endif

// Message buffer structure
struct a1buff_t
{
    uint8_t buf[MAX_BUF_LEN] = {};
    int nseg = 0;
    int nbyte = 0;
    int nlen = 0;
    int type = 0;
    int subtype = 0;
    int crc = 0;
    int loc[MAXFIELD] = {};
};

// RTCM binary message structures
#pragma pack(push, 1)

struct rtcm_apim1_t
{
    uint64_t MCU_Time;
    uint64_t Sync_Time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t Temp_C;
};

struct rtcm_apimu_t
{
    uint64_t MCU_Time;
    uint64_t Sync_Time;
    uint64_t ODO_time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t ODO;
    int16_t Temp_C;
};

struct rtcm_old_apimu_t
{
    uint64_t MCU_Time;
    uint64_t ODO_time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t ODO;
    int16_t Temp_C;
};

struct rtcm_apgps_t
{
    uint64_t Time;
    uint64_t GPS_Time;
    int32_t Latitude;
    int32_t Longitude;
    int32_t Alt_ellipsoid;
    int32_t Alt_msl;
    int32_t Speed;
    int32_t Heading;
    uint32_t Hor_Acc;
    uint32_t Ver_Acc;
    uint32_t Hdg_Acc;
    uint32_t Spd_Acc;
    uint16_t PDOP;
    uint8_t FixType;
    uint8_t SatNum;
    uint8_t RTK_Status;
    uint8_t Antenna_ID;
};

struct rtcm_aphdr_t
{
    uint64_t MCU_Time;
    uint64_t GPS_Time;
    int32_t relPosN;
    int32_t relPosE;
    int32_t resPosD;
    int32_t relPosLength;
    int32_t relPosHeading;
    uint32_t relPosLength_Accuracy;
    uint32_t relPosHeading_Accuracy;
    uint16_t statusFlags;
};

struct rtcm_apins_t
{
    uint64_t Time;
    uint64_t GPS_Time;
    int32_t Latitude;
    int32_t Longitude;
    int32_t Alt_ellipsoid;
    int32_t Vn, Ve, Vd;
    int32_t Roll, Pitch, Heading_Yaw;
    uint8_t ZUPT;
    uint8_t Status;
};

struct rtcm_apcov_t
{
    uint64_t Time;
    float covLatLat;
    float covLonLon;
    float covAltAlt;
    float covLatLon;
    float covLatAlt;
    float covLonAlt;
    float covVnVn;
    float covVeVe;
    float covVdVd;
    float covVnVe;
    float covVnVd;
    float covVeVd;
    float covRollRoll;
    float covPitchPitch;
    float covYawYaw;
    float covRollPitch;
    float covRollYaw;
    float covPitchYaw;
};

struct rtcm_apahrs_t {
    uint64_t Time, SyncTime;
    int32_t Roll, Pitch, Yaw;
    uint8_t ZUPT;
};

#pragma pack(pop)

#endif
