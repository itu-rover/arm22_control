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
      ss << std::setw(3) << std::setfill('0') << abs((int)v);
      return (v > 0 ? "1" : "0") + ss.str();
    };

    comm_check_bit ^= 1;
    std::string msg_to_send = "";
    msg_to_send += "S";
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[0], -3.14, 3.14, 999 / 2, -999 / 2), -250, 250));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[1], -1.57, 1.57, -999, 999), -350, 350));
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[2], 0, 1.0732421875, -999, 999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[3], -6.28, 6.28, 999, -999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[4], -1.57, 1.57, -999, 999), -999, 999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[5], -6.28, 6.28, 999, -999), -999, 999));
    msg_to_send += (comm_check_bit ? "1" : "0");
    msg_to_send += "F";
    ROS_INFO("Sending the msgx: %s  with a length of %ld", msg_to_send.c_str(), msg_to_send.size());
    serial_->write(msg_to_send);
    std::string result = serial_->readline(26, "B");

    ROS_INFO_THROTTLE(10, "Got encoder message: %s  with a length of %ld", result.c_str(), result.size());
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
      return (s[0] == '0' ? -1 : 1) * stoi(s.substr(1, 3));
    };

    int16_t axis1 = serial_to_int(serial_msg.substr(1, 4));
    int16_t axis2 = serial_to_int(serial_msg.substr(5, 4));
    int16_t axis3 = serial_to_int(serial_msg.substr(9, 4));
    int16_t axis4 = serial_to_int(serial_msg.substr(13, 4));
    int16_t axis5 = serial_to_int(serial_msg.substr(17, 4));
    int16_t axis6 = serial_to_int(serial_msg.substr(21, 4));

    double axis1_position = rover::map((double)axis1, -999, 999, 6.28, -6.28);
    double axis2_position = rover::map((double)axis2, -999, 999, -1.57, 1.57);
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    double axis3_position = rover::map((double)axis3, -999, 999, 0, 1.0732421875);
    double axis4_position = rover::map((double)axis4, -999, 999, 6.28, -6.28);
    double axis5_position = rover::map((double)axis5, -999, 999, -1.57, 1.57);
    double axis6_position = rover::map((double)axis6, -999, 999, 6.28, -6.28);

    joint_position_[0] = axis1_position;
    joint_position_[1] = axis2_position;
    joint_position_[2] = axis3_position;
    joint_position_[3] = axis4_position;
    joint_position_[4] = axis5_position;
    joint_position_[5] = axis6_position;
  }
}