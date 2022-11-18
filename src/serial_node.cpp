#include <cartbot/utility.h>
#include <cartbot/serialib.h>
#include <cartbot/serial.h>

serialib serial;

void requestFeedback(serialib &serial, const uint8_t id, const uint8_t mode)
{
    uint8_t tx_buff[6];
    uint8_t check_sum = 0;
    tx_buff[0] = HEADER1;
    tx_buff[1] = HEADER2;
    tx_buff[2] = id;
    tx_buff[3] = 0x02;
    tx_buff[5] = mode;
    for (std::size_t i = 2; i < sizeof(tx_buff); i++)
    {
        if (i == 4)
            continue;
        check_sum += tx_buff[i];
    }
    tx_buff[4] = ~check_sum;
    serial.writeBytes(tx_buff, sizeof(tx_buff));
}

void setSpeed(serialib &serial, const uint8_t id, const uint8_t dir, const uint16_t speed)
{
    uint8_t tx_buff[10];
    uint8_t check_sum = 0;
    tx_buff[0] = HEADER1;
    tx_buff[1] = HEADER2;
    tx_buff[2] = id;
    tx_buff[3] = 0x06;
    tx_buff[5] = MODE::ACCEL_VELOCIY_CONTROL;
    // direction
    tx_buff[6] = dir;
    // speed
    tx_buff[7] = speed >> 8;
    tx_buff[8] = speed & 0xFF;
    tx_buff[9] = 0x02;
    for (std::size_t i = 2; i < sizeof(tx_buff); i++)
    {
        if (i == 4)
            continue;
        check_sum += tx_buff[i];
    }
    tx_buff[4] = ~check_sum;
    serial.writeBytes(tx_buff, sizeof(tx_buff));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    if (serial.openDevice("/dev/ttyUSB0", 250000) != 1)
    {
        ROS_ERROR_STREAM("Can not open Serial /dev/ttyUSB0");
        exit(0);
    }
    ROS_INFO("\033----> Serial Commnicaiton Started.\033");
    while (ros::ok())
    {
        /*
           ID0 --> CCW : FORWARD / CW: BACKWARD
           ID1 --> CW : FORWARD  / CXW : BACKWARD
        */
        setSpeed(serial, 0, DIRECTION::CCW, 50);
        setSpeed(serial, 1, DIRECTION::CW, 50);
        requestFeedback(serial, 0, FEEDBACK::VELOCITY_FEEDBACK);
        uint8_t rx_buff[12];
        serial.readBytes(rx_buff, sizeof(rx_buff));
        for (int i = 0; i < sizeof(rx_buff); i++)
        {
            std::cout << std::hex << (int)rx_buff[i] << ", ";
        }
        std::cout << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}