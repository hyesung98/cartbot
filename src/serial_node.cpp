#include <cartbot/utility.h>
#include <cartbot/serialib.h>
#include <cartbot/serial.h>

ros::Publisher encoder_pub;
serialib serial;
uint16_t enc_cnt_l, enc_cnt_r;
int16_t tar_rpm_l = 0, tar_rpm_r = 0;
int16_t now_rpm_l = 0, now_rpm_r = 0;

uint8_t check_buff[4];
int state = 1;
int idx = 0;
int a = 0;
bool check = false;

void setSpeed(serialib &serial, const int16_t speed_l, const int16_t speed_r)
{
    uint16_t u_speed_r = static_cast<uint16_t>(abs(speed_r));
    uint16_t u_speed_l = static_cast<uint16_t>(abs(speed_l));
    uint8_t tx_buff[9];
    uint8_t check_sum = 0;
    tx_buff[0] = HEADER1;
    tx_buff[1] = HEADER2;
    /*
        ID0 --> CCW : FORWARD / CW: BACKWARD
        ID1 --> CW : FORWARD  / CCW : BACKWARD
    */
    if (speed_l < 0){
        tx_buff[2] = DIRECTION::CCW;
    }
    else
    {
        tx_buff[2] = DIRECTION::CW;
    }
    if (speed_r < 0){
        tx_buff[3] = DIRECTION::CCW;
    }
    else
    {
        tx_buff[3] = DIRECTION::CW;
    }
    tx_buff[4] = u_speed_l >> 8;
    tx_buff[5] = u_speed_l & 0xFF;
    tx_buff[6] = u_speed_r >> 8;
    tx_buff[7] = u_speed_r & 0xFF;
    for (std::size_t i = 2; i < sizeof(tx_buff) - 1; i++)
    {
        check_sum += tx_buff[i];
    }
    tx_buff[8] = ~check_sum;
    serial.writeBytes(tx_buff, 9);
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
    if (serial.openDevice("/dev/ttyUSB1", 115200) != 1)
    {
        ROS_ERROR_STREAM("Can not open Serial /dev/ttyUSB1");
        exit(0);
    }
    ROS_INFO("\033----> Serial Commnicaiton Started.\033");
    while (ros::ok())
    {
        setSpeed(serial, tar_rpm_l, tar_rpm_r);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
