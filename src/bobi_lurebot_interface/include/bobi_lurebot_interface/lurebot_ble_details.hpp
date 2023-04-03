#ifndef LUREBOT_DETAILS_H
#define LUREBOT_DETAILS_H

#include <stdint.h>
#include <numeric>

#define INFO_SRV_UUID "2a00970d-6592-4c13-bfc0-6eab4c021967"
#define DEVICE_NAME_CHAR_UUID "6a7d1ab3-8e30-44a3-ab60-adf2247233ff"
#define FW_VERSION_CHAR_UUID "f2fac9b2-6646-402a-a387-8aace42aec50"

#define MOTOR_VEL_SRV_UUID "577be3a5-d33d-4961-a2d0-eb0e48d6f0c8"
#define DESIRED_VEL_CHAR_UUID "1804a831-ce32-4481-affa-6de6754f9fd2"
#define RETURN_CURRENT_VEL_CHAR_UUID "b8c18dc6-1b80-4833-bad3-7f3411cdb25f"
#define CURRENT_VEL_CHAR_UUID "49a6d7a3-658a-4a3c-81e7-7a3360ecdaaf"

#define ACCEL_SRV_UUID "84f74144-6d58-4e96-98b0-c9eab0bcdc55"
#define MAX_ACCEL_CHAR_UUID "6e9942df-a2ec-477a-96da-79ec9b78ba9f"

#define DUTY_CYCLE_SRV_UUID "f19b954f-ff61-423c-bf6a-587a791b14a7"
#define SET_DUTY_CYCLE_CHAR_UUID "34a29f06-ef2f-433d-8749-3122b3259a82"

#define IR_SRV_UUID "dc8db6a1-eef8-489d-998c-348c2ad7405b"
#define SET_IR_CHAR_UUID "98db2590-628e-41ce-a0f6-6004e5306f79"
#define IR_VAL_CHAR_UUID "9eda0759-3d2a-4e76-805a-0ec007282868"

#define DROPPED_MSGS_SRV_UUID "fbbfb9b9-ce47-4ce8-9156-f1139560022e"
#define RETURN_DROPPED_MSGS_CHAR_UUID "312abc15-73cf-4a55-9f8f-f822a66f2bf7"
#define DROPPED_MSGS_COUNT_CHAR_UUID "e354a4a8-3202-401d-9bb6-2f918c257338"

#define HEARTBEAT_SRV_UUID "f506221e-4c45-4994-b3d0-9a78863c64b7"
#define HEARTBEAT_CHAR_UUID "1beb96cc-e84d-44dc-82be-0dbea32c1f4c"

#define TEMPERATURE_SRV_UUID "1d129153-2148-4770-90f2-277555ab7fd1"
#define RETURN_CURRENT_TEMP_CHAR_UUID "2990fb43-e65c-4543-adcd-2b57d0a9aeb8"
#define TEMPERATURE_CHAR_UUID "3488b1d2-85ab-4dee-aa63-2674c34de78b"

#define MAX_BUFFER_SIZE 20
#define DROPPED_MSG_BUFFER_SIZE 16
#define MOTOR_VEL_CHAR_SIZE 8
#define MOTOR_CVEL_CHAR_SIZE 4
#define IR_VAL_CHAR_SIZE 4
#define TEMP_BUFF_SIZE 2

#define LUREBOT_NAME_BUF_SIZE 20
#define FW_VERSION_BUF_SIZE 15

union MotorCmd {
    uint8_t bytes[MOTOR_VEL_CHAR_SIZE];
    uint16_t cmds[MOTOR_VEL_CHAR_SIZE / 2];
};

union DroppedMsg {
    uint8_t bytes[2 * DROPPED_MSG_BUFFER_SIZE];
    uint64_t counters[2];
};

union MotorSpeeds {
    uint8_t bytes[MOTOR_CVEL_CHAR_SIZE];
    uint16_t cmds[MOTOR_CVEL_CHAR_SIZE / 2];
};

union LurebotName {
    uint8_t bytes[LUREBOT_NAME_BUF_SIZE];
    char cbytes[LUREBOT_NAME_BUF_SIZE + 1];
};

union LurebotFWVersion {
    uint8_t bytes[FW_VERSION_BUF_SIZE];
    char cbytes[FW_VERSION_BUF_SIZE + 1];
};

union Temperature {
    uint8_t bytes[TEMP_BUFF_SIZE];
    uint16_t cmds[TEMP_BUFF_SIZE / 2];
};

template <typename UT, typename T>
UT toSigned(T val)
{
    T mid = (std::numeric_limits<T>::max() - 1) / 2 + 1;
    UT v = (val >= mid) ? (UT)(val - mid) : (UT)val - (UT)mid;
    return v;
}

template <typename UT, typename T>
UT toUnsigned(const T val)
{
    UT mid = (std::numeric_limits<UT>::max() - 1) / 2 + 1;
    return val + mid;
}

#endif