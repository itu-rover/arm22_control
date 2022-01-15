/**
 * @author B. Burak Payzun
 * @date 2022-01-01
 * 
 */

#include <arm21_control/arm21_hw_interface.h>
#include <rover_utils/math_helpers.h>

namespace arm21
{
  Arm21HWInterface::Arm21HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
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

  void Arm21HWInterface::enforceLimits(ros::Duration &period)
  {
    return;
  }

  void Arm21HWInterface::init()
  {
    ros_control_boilerplate::GenericHWInterface::init();

    ROS_INFO("Arm21HWInterface initialized.");
  }

  void Arm21HWInterface::read(ros::Duration &elapsed_time)
  {
    // TODO: Implement this function
    return;
  }

  void Arm21HWInterface::write(ros::Duration &elapsed_time)
  {
    static int count = 0;
    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg */
    /* Axis 4 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 5 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 6 takes values in between -999 and 999, corresponding to -180 deg to 180 deg */

    double axis_1_deg = joint_position_command_[0] * RAD_TO_DEG;
    double axis_2_deg = 85.7 + (joint_position_command_[1] * RAD_TO_DEG);
    double axis_3_deg = 92.6 - (joint_position_command_[2] * RAD_TO_DEG);
    double axis_4_deg = joint_position_command_[3] * RAD_TO_DEG;
    double axis_5_deg = joint_position_command_[4] * RAD_TO_DEG;
    double axis_6_deg = joint_position_command_[5] * RAD_TO_DEG;

    axis_1_deg = rover::clamp(axis_1_deg, -135.0, 135.0);
    axis_2_deg = rover::clamp(axis_2_deg, 24.2, 85.7);
    axis_3_deg = rover::clamp(axis_3_deg, 92.6, 145.3);
    axis_4_deg = rover::clamp(axis_4_deg, -90.0, 90.0);
    axis_5_deg = rover::clamp(axis_5_deg, -90.0, 90.0);
    axis_6_deg = rover::clamp(axis_6_deg, -180.0, 180.0);

    auto to_serial = [](double v)
    {
      std::stringstream ss;
      ss << std::setw(3) << std::setfill('0') << abs((int)v);
      return (v < 0 ? "1" : "0") + ss.str();
    };

    /*
    * Total size of 58 digits (Including S and F):
    * Index: 0 | Always starts with the letter 'S'
    * Index: 1 | 
    * 4 Digits 0000 Left speed
    * 4 Digits 0000 Right Speed
    * 4 Digits 0000 Axis 1 
    * 4 Digits 0000 Axis 2
    * 4 Digits 0000 Axis 3
    * 4 Digits 0000 Axis 4
    * 4 Digits 0000 Axis 5
    * 4 Digits 0000 Axis 6
    * 4 Digits 0000 Gripper
    * 
    * Blank
    *
    * 52 One Time Command
    * 53 Mode
    * 54 Arduino mode
    * 55 Servo mode arduino
    * 56 
    * Index: 57 | Always end with the letter 'F'
    */

    std::string msg_to_sent = "";
    msg_to_sent += "S";
    msg_to_sent += "0000"; // Left speed
    msg_to_sent += "0000"; // Right speed
    msg_to_sent += to_serial(rover::map(axis_1_deg, -135, 135, -999, 999));
    msg_to_sent += to_serial(rover::map(axis_2_deg, 24.2, 85.7, 999, 0));
    msg_to_sent += to_serial(rover::map(axis_3_deg, 92.6, 145.3, 0, -999));
    msg_to_sent += to_serial(rover::map(-axis_4_deg, -90, 90, -999, 999));
    msg_to_sent += to_serial(rover::map(-axis_5_deg, -90, 90, -999, 999));
    msg_to_sent += to_serial(rover::map(axis_6_deg, -180, 180, -999, 999));
    msg_to_sent += "0000"; // Gripper
    msg_to_sent += "0000000000000000400";
    msg_to_sent += (count < 5 ? "1" : "0");
    msg_to_sent += "F";

    // msg_to_sent = "S00000000000000000000000000000000000000000000000000005000F"; // Uncomment this to send handbrake voltage command
    ROS_INFO("Sending the msgx: %s  with a length of %ld", msg_to_sent.c_str(), msg_to_sent.size());
    serial_->write(msg_to_sent);
    std::string result = serial_->read(88);
    count++;
    count = count % 9;

    ROS_INFO("Read the msg: %s  with a length of %ld", result.c_str(), result.size());
    feedback(result);
  }

  void Arm21HWInterface::feedback(std::string serial_msg)
  {
    /* Assuming serial_msg has the following pattern:
      A,0000,0000,0000,0000,AXIS,AXIS,AXIS,AXIS,AXIS,AXIS,GRIP,0000000000000000000000000000 ,B
   */

    if (serial_msg.size() != 88)
      return;

    auto serial_to_int = [](const std::string &s)
    {
      ROS_ASSERT(s.size() == 4);
      return (s[0] == '0' ? 1 : -1) * stoi(s.substr(1, 3));
    };

    // Encoder values are in between -999 and 999
    int16_t axis1 = serial_to_int(serial_msg.substr(22, 4));
    int16_t axis2 = serial_to_int(serial_msg.substr(27, 4));
    int16_t axis3 = serial_to_int(serial_msg.substr(32, 4));
    int16_t axis4 = serial_to_int(serial_msg.substr(37, 4));
    int16_t axis5 = serial_to_int(serial_msg.substr(42, 4));
    int16_t axis6 = serial_to_int(serial_msg.substr(47, 4));

    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg */
    /* Axis 4 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 5 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 6 takes values in between -999 and 999, corresponding to -180 deg to 180 deg */

    double axis1_position = rover::map((double)axis1, -999, 999, -135 * DEG_TO_RAD, 135 * DEG_TO_RAD);
    double axis2_position = rover::map((double)axis2, 0, 999, 0, -(85.7 - 24.2) * DEG_TO_RAD);
    double axis3_position = rover::map((double)axis3, 0, 999, 0, -(145.3 - 92.6) * DEG_TO_RAD);
    double axis4_position = rover::map(-(double)axis4, -999, 999, -90 * DEG_TO_RAD, 90 * DEG_TO_RAD);
    double axis5_position = rover::map(-(double)axis5, -999, 999, -90 * DEG_TO_RAD, 90 * DEG_TO_RAD);
    double axis6_position = rover::map((double)axis6, -999, 999, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD);

    joint_position_[0] = axis1_position;
    joint_position_[1] = axis2_position;
    joint_position_[2] = axis3_position;
    joint_position_[3] = axis4_position;
    joint_position_[4] = axis5_position;
    joint_position_[5] = axis6_position;
  }
}