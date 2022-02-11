/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 * 
 */

#include <arm22_control/arm22_hw_interface.h>
#include <rover_utils/math_helpers.h>

namespace arm22
{
  arm22HWInterface::arm22HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    serial_ = new serial::Serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(200));
    if (serial_->isOpen())
    {
      ROS_INFO("Succesfully opened the serial port.");
    }
    else
    {
      ROS_INFO("Failed to open the serial port.");
    }
  }

  void arm22HWInterface::enforceLimits(ros::Duration &period)
  {
    return;
  }

  void arm22HWInterface::init()
  {
    ros_control_boilerplate::GenericHWInterface::init();

    ROS_INFO("arm22HWInterface initialized.");
  }

  void arm22HWInterface::read(ros::Duration &elapsed_time)
  {
    // TODO: Implement this function
    return;
  }

  void arm22HWInterface::write(ros::Duration &elapsed_time)
  {
    static int count = 0;
    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg */
    /* Axis 4 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 5 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 6 takes values in between -999 and 999, corresponding to -180 deg to 180 deg */


    auto to_serial = [](double v)
    {
      std::stringstream ss;
      ss << std::setw(3) << std::setfill('0') << abs((int)v);
      return (v > 0 ? "1" : "0") + ss.str();
    };

    std::string msg_to_send = "";
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[0], -3.14, 3.14, 999/2, -999/2), -250, 250));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[1], -1.57, 1.57, -999, 999), -350, 350));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[2], 0, 2.146484375, -999, 999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[3], -6.28, 6.28, 999, -999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[4], -1.57, 1.57, -999, 999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[5], -6.28, 6.28, 999, -999), -999, 999));

    
    ROS_INFO("New serial: %s", msg_to_send.c_str());
    count ^= 1;

    std::string msg_to_sent = "";
    msg_to_sent += "S";
    // msg_to_sent = "S110001500700160007000800";
    // msg_to_sent = "S000000000999170000000700";
    msg_to_sent += msg_to_send;
    // msg_to_sent = "S000000000999000000000000";

    msg_to_sent += (count ? "1" : "0");
    msg_to_sent += "F";
    // msg_to_sent = "S00000000000000000000000000000000000000000000000000005000F"; // Uncomment this to send handbrake voltage command
    ROS_INFO("Sending the msgx: %s  with a length of %ld", msg_to_sent.c_str(), msg_to_sent.size());
    serial_->write(msg_to_sent);
    std::string result = serial_->readline(26, "B");

    ROS_INFO("Read the msg: %s  with a length of %ld", result.c_str(), result.size());
    feedback(result);
  }

  void arm22HWInterface::feedback(std::string serial_msg)
  {
    /* Assuming serial_msg has the following pattern:
      A,0000,0000,0000,0000,AXIS,AXIS,AXIS,AXIS,AXIS,AXIS,GRIP,0000000000000000000000000000 ,B
   */
    if (serial_msg.size() != 26)
      return;

    auto serial_to_int = [](const std::string &s)
    {
      ROS_ASSERT(s.size() == 4);
      return (s[0] == '0' ? -1 : 1) * stoi(s.substr(1, 3));
    };

    // Encoder values are in between -999 and 999
    ROS_INFO("encoder message: %s", serial_msg.c_str());
    int16_t axis1 = serial_to_int(serial_msg.substr(1, 4));
    int16_t axis2 = serial_to_int(serial_msg.substr(5, 4));
    int16_t axis3 = serial_to_int(serial_msg.substr(9, 4));
    int16_t axis4 = serial_to_int(serial_msg.substr(13, 4));
    int16_t axis5 = serial_to_int(serial_msg.substr(17, 4));
    int16_t axis6 = serial_to_int(serial_msg.substr(21, 4));

    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg */
    /* Axis 4 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 5 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 6 takes values in between -999 and 999, corresponding to -180 deg to 180 deg */
    ROS_INFO("%d %d %d", axis4, axis5, axis6); 
    double axis1_position = rover::map((double)axis1, -999, 999, 6.28, -6.28);
    double axis2_position = rover::map((double)axis2, -999, 999, -1.57, 1.57); 
    double axis3_position = rover::map((double)axis3, -999, 999, 0, 2.146484375); // where 1.2.146484375 = 3.14 * 700/1024  pi = 1024
    double axis4_position = rover::map((double)axis4, -999, 999, 6.28, -6.28);  
    double axis5_position = rover::map((double)axis5, -999, 999, -1.57, 1.57);  
    double axis6_position = rover::map((double)axis6, -999, 999, 6.28, -6.28); 

    ROS_INFO("%lf %lf %lf %lf %lf %lf", axis1_position, axis2_position, axis3_position, axis4_position, axis5_position, axis6_position); 
    joint_position_[0] = axis1_position;
    joint_position_[1] = axis2_position;
    joint_position_[2] = axis3_position;
    joint_position_[3] = axis4_position;
    joint_position_[4] = axis5_position;
    joint_position_[5] = axis6_position;
  }
}