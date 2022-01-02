/**
 * @author B. Burak Payzun
 * @date 2022-01-02
 * 
 */

#include <arm21_control/arm21_sim_interface.h>
#include <arm21_control/serial.h>
#include <sstream>
#include <iomanip>
#include <rover_utils/math_helpers.h>

void hello()
{
  ros::Rate rate(50);
  while (true)
  {
    ROS_INFO("HELLO");
    rate.sleep();
  }
}

namespace arm21
{
  Arm21SimInterface::Arm21SimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    _client = nh.serviceClient<arm21_control::serial>("/serial_echo");
    rx_thread = std::thread(hello);
  }

  void Arm21SimInterface::init()
  {
    ros_control_boilerplate::GenericHWInterface::init();

    ROS_INFO("Arm21 Sim Interface initialized.");
  }

  void Arm21SimInterface::read(ros::Duration &elapsed_time)
  {
    // TODO: Implement this function
    return;
  }

  void Arm21SimInterface::write(ros::Duration &elapsed_time)
  {
    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg 
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

    std::string msg_to_sent = "";
    msg_to_sent += "S";
    msg_to_sent += to_serial(rover::map(axis_1_deg, -135, 135, -999, 999));
    msg_to_sent += to_serial(rover::map(axis_2_deg, 24.2, 85.7, 999, 0));
    msg_to_sent += to_serial(rover::map(axis_3_deg, 92.6, 145.3, 0, 999));
    msg_to_sent += to_serial(rover::map(axis_4_deg, -90, 90, -999, 999));
    msg_to_sent += to_serial(rover::map(axis_5_deg, -90, 90, -999, 999));
    msg_to_sent += to_serial(rover::map(axis_6_deg, -180, 180, -999, 999));
    msg_to_sent += "F";

    arm21_control::serial srv;
    srv.request.serial_msg = msg_to_sent;
    if (_client.call(srv))
    {

      ROS_INFO("Got encoder msg: %s", srv.response.encoder_msg.c_str());
    }

    feedback(std::string(srv.response.encoder_msg.c_str()));
  }

  void Arm21SimInterface::enforceLimits(ros::Duration &period)
  {
    // TODO: Implement this function
    return;
  }

  void Arm21SimInterface::feedback(std::string serial_msg)
  {
    /* Assuming serial_msg has the following pattern:
    for  S0000111122223333444455556666F

            Offset  Description
    1byte   0       Start byte 'F'
    4bytes  1       Axis 1 
    4bytes  5       Axis 2
    4bytes  9       Axis 3
    4bytes  13      Axis 4
    4bytes  17      Axis 5
    4bytes  21      Axis 6
    1byte   25      End byte 'F'

   */

    if (serial_msg.size() != 26)
      return;

    auto serial_to_int = [](const std::string &s)
    {
      ROS_ASSERT(s.size() == 4);
      return (s[0] == '0' ? 1 : -1) * stoi(s.substr(1, 3));
    };

    // Encoder values are in between -999 and 999
    int16_t axis1 = serial_to_int(serial_msg.substr(1, 4));
    int16_t axis2 = serial_to_int(serial_msg.substr(5, 4));
    int16_t axis3 = serial_to_int(serial_msg.substr(9, 4));
    int16_t axis4 = serial_to_int(serial_msg.substr(13, 4));
    int16_t axis5 = serial_to_int(serial_msg.substr(17, 4));
    int16_t axis6 = serial_to_int(serial_msg.substr(21, 4));

    /* Axis 1 takes values in between -999 and 999, corresponding to -135 deg to 135 deg */
    /* Axis 2 takes values in between 0 and 999, corresponding to 85.7 deg to 24.2 deg */
    /* Axis 3 takes values in between 0 and 999, corresponding to 92.6 deg to 145.3 deg 
  /* Axis 4 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 5 takes values in between -999 and 999, corresponding to -90 deg to 90 deg */
    /* Axis 6 takes values in between -999 and 999, corresponding to -180 deg to 180 deg */

    double axis1_position = rover::map((double)axis1, -999, 999, -135 * DEG_TO_RAD, 135 * DEG_TO_RAD);
    double axis2_position = rover::map((double)axis2, 0, 999, 0, -(85.7 - 24.2) * DEG_TO_RAD);
    double axis3_position = rover::map((double)axis3, 0, 999, 0, -(145.3 - 92.6) * DEG_TO_RAD);
    double axis4_position = rover::map((double)axis4, -999, 999, -90 * DEG_TO_RAD, 90 * DEG_TO_RAD);
    double axis5_position = rover::map((double)axis5, -999, 999, -90 * DEG_TO_RAD, 90 * DEG_TO_RAD);
    double axis6_position = rover::map((double)axis6, -999, 999, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD);

    joint_position_[0] = axis1_position;
    joint_position_[1] = axis2_position;
    joint_position_[2] = axis3_position;
    joint_position_[3] = axis4_position;
    joint_position_[4] = axis5_position;
    joint_position_[5] = axis6_position;
  }
} //namespace arm21