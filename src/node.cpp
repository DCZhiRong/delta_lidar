/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*   Author: 3iRobotics, Data:2017-09-15
*
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
//#ifndef _countof
//#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
//#endif
#define PORT_DEFAULT "/dev/ttyUSB0"           // Serial device driver name (sym link to real dev)
#define BAUD_RATE_DEFAULT 115200              // Serial baud rate
#define FRAME_ID_DEFAULT "laser_frame"        // frame_id in LaserScan messages
typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
} RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;

void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,
                  _rslidar_data *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = frame_id;
    scan_msg->angle_min = angle_min;
    scan_msg->angle_max = angle_max;
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (360.0f - 1.0f);

    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_time / (double)(node_count - 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = 5.0;

    scan_msg->ranges.resize(360, std::numeric_limits<float>::infinity());
    scan_msg->intensities.resize(360, 0.0);

    // Unpack data
    for (size_t i = 0; i < node_count; i++)
    {
        size_t current_angle = floor(nodes[i].angle);
        if (current_angle > 360.0)
        {
            printf("Lidar angle is out of range %d\n", (int)current_angle);
            continue;
        }
        float read_value = (float)nodes[i].distance;
        if (read_value < scan_msg->range_min || read_value > scan_msg->range_max)
            scan_msg->ranges[360 - 1 - current_angle] = std::numeric_limits<float>::infinity();
        else
            scan_msg->ranges[360 - 1 - current_angle] = read_value;

        float intensities = (float)nodes[i].signal;
        scan_msg->intensities[360 - 1 - current_angle] = intensities;
    }

    pub->publish(std::move(scan_msg));
}

int main(int argc, char *argv[])
{
    // read ros param
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("delta_2b_lidar_node");



    rclcpp::NodeOptions options;
    auto nh_private = std::make_shared<rclcpp::Node>("delta_2b_lidar_node", options);
    node->declare_parameter("port", PORT_DEFAULT);
    auto port_param = rclcpp::Parameter("port", PORT_DEFAULT);

    node->declare_parameter("baud_rate", BAUD_RATE_DEFAULT);
    auto baud_rate_param = rclcpp::Parameter("baud_rate", BAUD_RATE_DEFAULT);

    node->declare_parameter("frame_id", FRAME_ID_DEFAULT);
    auto frame_id_param  = rclcpp::Parameter("frame_id", FRAME_ID_DEFAULT);

        
    node->get_parameter_or("port", port_param, port_param);
    node->get_parameter_or("baud_rate", baud_rate_param, baud_rate_param);
    node->get_parameter_or("frame_id", frame_id_param, frame_id_param);
    string opt_com_path = port_param.value_to_string();
    int opt_com_baudrate = baud_rate_param.as_int();
    string frame_id = frame_id_param.value_to_string();

    //    nh_private.param<string>("lidar_scan", lidar_scan, "scan");
    //    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(lidar_scan, 1000);
    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if (serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serial port successful!\n");
    }
    else
    {
        printf("[AuxCtrl] Open serial port %s failed! \n", opt_com_path.c_str());
        return -1;
    }

    printf("3iRoboticsLidar connected\n");

    robotics_lidar.initilize(&serial_connect);

    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;

    start_scan_time = node->now();

    while (rclcpp::ok())
    {
        TLidarGrabResult result = robotics_lidar.getScanData();
        switch (result)
        {
        case LIDAR_GRAB_ING:
        {
            break;
        }
        case LIDAR_GRAB_SUCESS:
        {
            TLidarScan lidar_scan = robotics_lidar.getLidarScan();
            size_t lidar_scan_size = lidar_scan.getSize();
            std::vector<RslidarDataComplete> send_lidar_scan_data;
            send_lidar_scan_data.resize(lidar_scan_size);
            RslidarDataComplete one_lidar_data;
            for (size_t i = 0; i < lidar_scan_size; i++)
            {
                one_lidar_data.signal = lidar_scan.signal[i];
                one_lidar_data.angle = lidar_scan.angle[i];
                one_lidar_data.distance = lidar_scan.distance[i];
                send_lidar_scan_data[i] = one_lidar_data;
            }

            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            end_scan_time = node->now();
            scan_duration = (end_scan_time - start_scan_time).seconds();
            printf("Receive Lidar count %u!\n", (unsigned int)lidar_scan_size);

            // if successful, publish lidar scan
            int start_node = 0, end_node = 359;
            publish_scan(scan_pub, &send_lidar_scan_data[0], lidar_scan_size,
                         start_scan_time, scan_duration,
                         angle_min, angle_max,
                         frame_id);

            start_scan_time = end_scan_time;

            break;
        }
        case LIDAR_GRAB_ERRO:
        {
            break;
        }
        case LIDAR_GRAB_ELSE:
        {
            printf("[Main] LIDAR_GRAB_ELSE!\n");
            break;
        }
        }
        //usleep(50);
        //rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}

