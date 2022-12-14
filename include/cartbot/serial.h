#ifndef SERIAL_H
#define SERIAL_H

#define MAX_BUFF 11
#define HEADER1 0xFF
#define HEADER2 0xFE

enum MODE
{
    POSTION_CONTROL = 0x01,
    ACCEL_POSTION_CONTROL = 0x02,
    ACCEL_VELOCIY_CONTROL = 0x03,
    SET_POSTION_CONTROLLER = 0x04,
    SET_VELOCITY_CONTROLLER = 0x05,
    SET_ID = 0x06,
    SET_COMMNICATION_SPEED = 0x07,
    SET_RESPONSE_TIME = 0x08,
    SET_RATED_SPEED = 0x09,
    SET_RESOLUTION = 0x0A,
    SET_REDUCTION_RATIO = 0x0B,
    SET_ONOFF = 0x0C,
    SET_POSITION_CONTROL_MODE = 0x0D,
    SET_CONTROL_DIRECTION = 0X0E,
    RESET_POSITION = 0x0F,
    RESET_CONTROLLER = 0x10
};

enum FEEDBACK
{
    PING = 0xA0,
    POSITON_FEEDBACK = 0XA1,
    VELOCITY_FEEDBACK = 0xA2,
    POSITION_CONTROLLER_FEEDBACK = 0xA3,
    VELOCITY_CONTROLLER_FEEDBACK = 0xA3,
    RESPONSE_TIME_FEEDBACK = 0xA5,
    RATED_SPEED_FEEDBACK = 0xA6,
    RESOLUTION_FEEDBACK = 0xA7,
    REDUCTION_RATIO_FEEDBACK = 0xA8,
    ONOFF_FEEDBACK = 0xA9,
    POSITION_CONTROL_FEEDBACK = 0xAA,
    CONTROL_DIRECTION_FEEDBACK = 0xAB,
    FW_VERISON_FEEDBACK = 0XCD
};

enum DIRECTION
{
    CCW = 0x00,
    CW = 0x01
};

enum BAUDRATE
{
    BAUD110 = 0x00,
    BAUD300 = 0x01,
    BAUD600 = 0x02,
    BAUD1200 = 0x03,
    BAUD2400 = 0x04,
    BAUD4800 = 0x05,
    BAUD9600 = 0x06,
    BAUD14400 = 0x07,
    BAUD19200 = 0x08,
    BAUD28800 = 0x09,
    BAUD38400 = 0x0A,
    BAUD57600 = 0x0B,
    BAUD76800 = 0x0C,
    BAUD115200 = 0x0D,
    BAUD230400 = 0x0E,
    BAUD250000 = 0x0F
};

#endif