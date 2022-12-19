#include <cartbot/utility.h>
#include <cartbot/serialib.h>
#include <cartbot/serial.h>

ros::Publisher encoder_pub;
serialib serial;
uint16_t enc_cnt_l, enc_cnt_r;
int16_t tar_rpm_l = 0, tar_rpm_r = 0;
int16_t now_rpm_l = 0, now_rpm_r = 0;
uint8_t rx_buff[1];
uint8_t check_buff[4];
uint8_t check_sum = 0;
int state = 1;
int idx = 0;
int a = 0;
bool check = false;

// void setSpeed(serialib &serial, const int16_t speed_l, const int16_t speed_r)
// {
//     uint16_t u_speed_r = static_cast<uint16_t>(abs(speed_r));
//     uint16_t u_speed_l = static_cast<uint16_t>(abs(speed_l));
//     uint8_t tx_buff[9];
//     uint8_t check_sum = 0;
//     tx_buff[0] = HEADER1;
//     tx_buff[1] = HEADER2;
//     /*
//         ID0 --> CCW : FORWARD / CW: BACKWARD
//         ID1 --> CW : FORWARD  / CCW : BACKWARD
//     */
//     if (speed_l < 0){
//         tx_buff[2] = DIRECTION::CCW;
//     }
//     else
//     {
//         tx_buff[2] = DIRECTION::CW;
//     }
//     if (speed_r < 0){
//         tx_buff[3] = DIRECTION::CCW;
//     }
//     else
//     {
//         tx_buff[3] = DIRECTION::CW;
//     }
//     tx_buff[4] = u_speed_l >> 8;
//     tx_buff[5] = u_speed_l & 0xFF;
//     tx_buff[6] = u_speed_r >> 8;
//     tx_buff[7] = u_speed_r & 0xFF;
//     for (std::size_t i = 2; i < sizeof(tx_buff) - 1; i++)
//     {
//         check_sum += tx_buff[i];
//     }
//     tx_buff[8] = ~check_sum;
//     serial.writeBytes(tx_buff, 9);
// }

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



bool getSpeed(serialib &serial, uint16_t &cnt_l, uint16_t &cnt_r)
{

    uint16_t rx_cnt_l = 0, rx_cnt_r = 0;

    for (int i = 0; i < sizeof(rx_buff); i++)
    {
        std::cout << std::hex << (int)rx_buff[i] << ", ";
    }

    // if (rx_buff[0] == HEADER1 && rx_buff[1] == HEADER2)
    // {
    //     for (size_t i = 2; i < sizeof(rx_buff) - 1; i++)
    //     {
    //         check_sum += rx_buff[i];
    //     }
    //     if (check_sum == rx_buff[6])
    //     {
    //         rx_cnt_l = rx_buff[2];
    //         rx_cnt_l = rx_cnt_l << 8;
    //         rx_cnt_l = rx_cnt_l || rx_buff[3];

    //         rx_cnt_r = rx_buff[4];
    //         rx_cnt_r = rx_cnt_r << 8;
    //         rx_cnt_r = rx_cnt_r || rx_buff[5];

    //         cnt_l = rx_cnt_l;
    //         cnt_r = rx_cnt_r;
    //         std::cout << "check_sum" << std::endl;
    //         return true;
    //     }
    // }
    return false;
}

void rpmCallback(const cartbot::Speed::ConstPtr &rpm_msg)
{
    tar_rpm_l = rpm_msg->tar_rpm_l;
    tar_rpm_r = rpm_msg->tar_rpm_r;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Subscriber tar_rpm_sub = nh.subscribe<cartbot::Speed>("/avoid/speed", 1, rpmCallback);
    encoder_pub = nh.advertise<cartbot::Encoder>("/serial/encoder", 1);
    if (serial.openDevice("/dev/tty485", 250000) != 1)
    {
        ROS_ERROR_STREAM("Can not open Serial /dev/tty485");
        exit(0);
    }
    // if (serial.openDevice("/dev/ttyUSB0", 115200) != 1)
    // {
    //     ROS_ERROR_STREAM("Can not open Serial /dev/ttyUSB1");
    //     exit(0);
    // }
    ROS_INFO("\033----> Serial Commnicaiton Started.\033");
    while (ros::ok())
    {
        // setSpeed(serial, tar_rpm_l, tar_rpm_r);
        setSpeed(serial, 0, 1, 200);
        setSpeed(serial, 1, 1, 200);
        // setSpeed(serial, -300, -300);
        // serial.readBytes(rx_buff, sizeof(rx_buff));
        // switch (state)
        // {
        // case 1:
        //     std::cout << std::hex << (int)rx_buff[0] << ", ";
        //     if (rx_buff[0] == HEADER1)
        //         state++;
        //     break;
        // case 2:
        //     std::cout << std::hex << (int)rx_buff[0] << ", ";
        //     if (rx_buff[0] == HEADER2)
        //         state++;
        //     break;
        // case 3:
        //     std::cout << std::hex << (int)rx_buff[0] << ", ";
        //     if (idx >= 4)
        //     {
        //         for (int i = 0; i < sizeof(check_buff); i++)
        //         {
        //             check_sum += check_buff[i];
        //         }
        //         if (rx_buff[0] == ~check_sum)
        //         {
        //             std::cout << std::endl;
        //             state = 1;
        //             idx = 0;
        //             check_sum = 0;
        //         }
        //     }
        //     check_buff[idx++] = rx_buff[0];
        //     break;
        // }

        // getSpeed(serial, enc_cnt_l, enc_cnt_r);
        // std::cout << "left: " << enc_cnt_l << std::endl;
        // std::cout << "right: " << enc_cnt_r << std::endl;

        // cartbot::Encoder encoder_data;
        // encoder_data.enc_cnt_l = enc_cnt_l;
        // encoder_data.enc_cnt_r = enc_cnt_r;
        // encoder_pub.publish(encoder_data);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}