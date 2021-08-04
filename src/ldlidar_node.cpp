#include <ldlidar_node.h>

LD06::LD06()
  : Node("ld06_node")
{
  topic_name_ = this->declare_parameter("topic_name", "scan");
  port_name_ = this->declare_parameter("port_name", "/dev/ttyACM0");
  lidar_frame_ = this->declare_parameter("lidar_frame", "laser");
  
  lidar_ = new LiPkg;
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

  CmdInterfaceLinux cmd_port;
  std::string port_name_;
  std::string lidar_frame_;

  lidar_->SetLidarFrame(lidar_frame_);

  if (port_name_.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autodetecting serial port");
    std::vector<std::pair<std::string, std::string>> device_list;
    cmd_port.GetCmdDevices(device_list);
    auto found = std::find_if(
      device_list.begin(),
      device_list.end(),
      [](std::pair<std::string, std::string> n)
      { return strstr(n.second.c_str(), "CP2102"); }
    );

    if (found != device_list.end())
    {
      port_name_ = found->first;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't find LiDAR LD06");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using port %s", port_name_);

  cmd_port.SetReadCallback(
    [this](const char *byte, size_t len) 
    {
      if(lidar_->Parse((uint8_t*)byte, len))
      {
        lidar_->AssemblePacket();
      }
    }
  );

  if(cmd_port.Open(port_name_))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LiDAR_LD06 started successfully");
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't open the serial port");
  }

  loop_timer_ = this->create_wall_timer(
    1ms, 
    std::bind(&LD06::publishLoop, this)
  );
}

void LD06::publishLoop()
{
  if (lidar_->IsFrameReady())
  {
    lidar_pub_->publish(lidar_->GetLaserScan());
    lidar_->ResetFrameReady();
  }
}