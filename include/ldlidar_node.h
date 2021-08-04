#ifndef LD06_H
#define LD06_H

#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tofbf.h"
#include <string>

using namespace std::chrono_literals;

class LD06 : public rclcpp::Node
{
  public:
    LD06();

  private:
    std::string topic_name_;
    std::string lidar_frame_;
    std::string port_name_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    LiPkg * lidar_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    void publishLoop();
};

#endif