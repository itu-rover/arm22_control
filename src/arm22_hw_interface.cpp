/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 *
 */

#include <arm22_control/arm22_hw_interface.h>
#include <rover_utils/math_helpers.h>
#include <string>

namespace arm22
{
  arm22HWInterface::arm22HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    // Read settings.yaml
    nh.param<std::string>("/serial/port", this->port, "/please_fill/settings.yaml");
    nh.param("/serial/baudrate", this->baudrate, 1);
    nh.param("/serial/encoder_delta_threshold", this->encoder_delta_threshold, 0.1);
    try
    {
      ROS_INFO("Trying to connect port: %s, baudrate: %d", this->port.c_str(), this->baudrate);
      serial_ = new serial::Serial(this->port, this->baudrate, serial::Timeout::simpleTimeout(200));
      if (serial_->isOpen())
      {
        ROS_INFO("Succesfully opened the serial port.");
      }
      else
      {
        ROS_ERROR("Failed to open the serial port.");
        ros::shutdown();
      }
    }
    catch (serial::IOException &e)
    {
      switch (e.getErrorNumber())
      {
      case 2:
        ROS_ERROR("Failed to initiate serial, no such file, check if the serial cable is connected.");
        break;
      case 13:
        ROS_ERROR("Failed to initiate serial, check permission.");
        break;
      default:
        ROS_ERROR("Failed to initiate serial.");
        ROS_ERROR("ERROR: %s", e.what());
        break;
      }

      ROS_ERROR("SHUTTING DOWN!");
      ros::shutdown();
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
    static int comm_check_bit = 0;
    static auto to_serial = [](double v)
    {
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << abs((int)v);
      return ss.str();
    };

    comm_check_bit ^= 1;
    std::string msg_to_send = "";
    msg_to_send += "S";
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[0], 3.14, -3.14, 0, 4096), 0, 4096));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[1],-1.57, 1.57/2, 0, 1024 + 512), 512, 1024+512));
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[2], 0, 1.57, 0, 1024), 0, 1024));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[3], 6.28, -6.28, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[4], -1.57, 1.57, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[5], 6.28, -6.28, 0, 9999), 0, 9999));
    msg_to_send += (comm_check_bit ? "1" : "0");
    msg_to_send += "F";
    for (double &v : joint_position_command_){
      ROS_INFO("%lf", v);
    }
    ROS_INFO("Sending the msg: %s  with a length of %ld", msg_to_send.c_str(), msg_to_send.size());
    serial_->write(msg_to_send);
    ROS_INFO("%s", msg_to_send.c_str());
    std::string result = serial_->readline(26, "B");

    ROS_INFO("Got encoder message: %s  with a length of %ld", result.c_str(), result.size());
    feedback(result);
  }

  void arm22HWInterface::feedback(std::string serial_msg)
  {
    /* Assuming serial_msg has the following pattern:

      A AXIS AXIS AXIS AXIS AXIS AXIS B
      total length = 26

    */

    if (serial_msg.size() != 26)
    {
      ROS_WARN("Encoder message with unexpected size: %ld bytes, [%s]", serial_msg.size(), serial_msg.c_str());
      return;
    }

    static auto serial_to_int = [](const std::string &s)
    {
      ROS_ASSERT(s.size() == 4);
      return stoi(s);
    };

    try{
      int16_t axis1 = serial_to_int(serial_msg.substr(1, 4));
      int16_t axis2 = serial_to_int(serial_msg.substr(5, 4));
      int16_t axis3 = serial_to_int(serial_msg.substr(9, 4));
      int16_t axis4 = serial_to_int(serial_msg.substr(13, 4));
      int16_t axis5 = serial_to_int(serial_msg.substr(17, 4));
      int16_t axis6 = serial_to_int(serial_msg.substr(21, 4));

      double axis1_position = rover::map((double)axis1, 0, 4096, 3.14, -3.14);
      double axis2_position = rover::map((double)axis2, 0, 1024+512, -1.57, 1.57/2);
      // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
      double axis3_position = rover::map((double)axis3, 0, 1024, 0, 1.57);
      double axis4_position = rover::map((double)axis4, 0, 9999, 6.28, -6.28);
      double axis5_position = rover::map((double)axis5, 0, 9999, -1.57, 1.57);
      double axis6_position = rover::map((double)axis6, 0, 9999, 6.28, -6.28);

      if(fabs(axis1_position - joint_position_[0]) > encoder_delta_threshold ||
          fabs(axis2_position - joint_position_[1]) > encoder_delta_threshold ||
          fabs(axis3_position - joint_position_[2]) > encoder_delta_threshold ||
          fabs(axis4_position - joint_position_[3]) > encoder_delta_threshold ||
          fabs(axis5_position - joint_position_[4]) > encoder_delta_threshold ||
          fabs(axis6_position - joint_position_[5]) > encoder_delta_threshold){
            while(true){
              ROS_ERROR("Encoder got a message with unexpected change. %lf %lf %lf %lf %lf %lf", axis1_position, axis2_position, axis3_position, axis4_position, axis5_position, axis6_position);
              serial_->write("S555555F");
              ros::Duration(5).sleep();
            }
        }
      joint_position_[0] = axis1_position;
      joint_position_[1] = axis2_position;
      joint_position_[2] = axis3_position;
      joint_position_[3] = axis4_position;
      joint_position_[4] = axis5_position;
      joint_position_[5] = axis6_position;
    }
    catch(std::invalid_argument &e){
      return;
    }
  }
}