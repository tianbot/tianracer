#ifndef _PROTOCOL_
#define _PROTOCOL_

#define PROTOCOL_HEAD 0xAA55

enum {
    PACK_TYPE_HEART_BEAT = 0x0000,
    PACK_TYPE_CMD_VEL,
    PACK_TYPE_ACKERMANN_CMD,
    PACK_TYPE_ODOM_RESPONSE = 0x8000,
    PACK_TYPE_UWB_RESPONSE,
    PACK_TYPE_HEART_BEAT_RESPONSE,
    PACK_TYPE_IMU_REPONSE
};

#pragma pack(push)
#pragma pack(1)

struct vector3 {
    float x;
    float y;
    float z;
};

struct twist {
    struct vector3 linear;
    struct vector3 angular;
};

struct quaternion {
    float x;
    float y;
    float z;
    float w;
};

struct pose {
    struct vector3 point;
    float yaw;
};

struct odom {
    struct pose pose;
    struct twist twist;
};

struct uwb
{
  float x_m;
  float y_m;
  float yaw;
  //uint32_t sig_level;
};

struct imu_feedback{
    struct quaternion quat;
    struct vector3 linear_acc;
    struct vector3 angular_vel;
};

struct ackermann_cmd{
    float steering_angle;
    float speed;
};

struct protocol_pack{
    uint16_t head;
    uint16_t len;   //data len + 2 byte pack_type
    uint16_t pack_type;
    uint8_t data[]; //contain bcc byte
};

#pragma pack(pop)
#endif
