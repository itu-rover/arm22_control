/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 * 
 */

#include <arm22_control/arm22_sim_interface.h>
#include <arm22_control/serial.h>
#include <sstream>
#include <iomanip>
#include <rover_utils/math_helpers.h>

bool is_number(std::string str){
  for(int i = 0; i < str.length(); i++){
    if(str[i] >= '0' && str[i] <= '9') continue;
    else return false;
  }
  return true;
}
namespace arm22
{
  arm22SimInterface::arm22SimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    _client = nh.serviceClient<arm22_control::serial>("/serial_echo");
    write_toggle_service_ = nh.advertiseService("/hardware_interface/toggle_write", &arm22SimInterface::toggle_write, this);
  }

  void arm22SimInterface::init()
  {
    ros_control_boilerplate::GenericHWInterface::init();

    ROS_INFO("arm22 Sim Interface initialized.");
  }

  void arm22SimInterface::read(ros::Duration &elapsed_time)
  {
    // TODO: Implement this function
    return;
  }

  void arm22SimInterface::write(ros::Duration &elapsed_time)
  {

    static auto to_serial = [](double v)
    {
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << abs((int)v);
      return ss.str();
    };

    static int comm_check_bit = 0;
    comm_check_bit ^= 1;

    std::string msg_to_send = "";
    msg_to_send += "S";
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[0], 3.14, -3.14, 0, 4096), 0, 4096));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[1],-1.57, 1.57/2, 0, 1024 + 512), 512, 1024+512));
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[2], -1.57/2, 1.57/2, 512, 1024+512), 512, 1024+512));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[3], 6.28, -6.28, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[4], -1.57, 1.57, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[5], -6.28, 6.28, 0, 9999), 0, 9999));
    msg_to_send += (comm_check_bit ? "1" : "0");
    msg_to_send += "F";

    arm22_control::serial srv;
    srv.request.serial_msg = msg_to_send;
    if (_client.call(srv))
    {

      ROS_INFO("Got encoder msg: %s", srv.response.encoder_msg.c_str());
    }

    feedback(std::string(srv.response.encoder_msg.c_str()));
  }

  bool arm22SimInterface::toggle_write(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){
    res.success = true;
    res.message = "";
    return true;
  }

  void arm22SimInterface::enforceLimits(ros::Duration &period)
  {
    // TODO: Implement this function
    return;
  }

  void arm22SimInterface::feedback(std::string serial_msg)
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

    if (serial_msg.size() != 27)
    {
      ROS_WARN("Encoder message with unexpected size: %ld bytes, [%s]", serial_msg.size(), serial_msg.c_str());
      return;
    }

    if (!is_number(serial_msg.substr(1,24)))
    {
      ROS_WARN("Encoder message with non-number character: %ld bytes, [%s]", serial_msg.size(), serial_msg.c_str());
      return;
    }

    if (serial_msg.rfind("S000000000000000000000000", 0) == 0){
      ROS_WARN("Encoder message is all zeros: %ld bytes, [%s]\n You might want to reset encoder positions.", serial_msg.size(), serial_msg.c_str());
      return;
    }

    static auto serial_to_int = [](const std::string &s)
    {
      ROS_ASSERT(s.size() == 4);
      return stoi(s);
    };

    int16_t axis1 = serial_to_int(serial_msg.substr(1, 4));
    int16_t axis2 = serial_to_int(serial_msg.substr(5, 4));
    int16_t axis3 = serial_to_int(serial_msg.substr(9, 4));
    int16_t axis4 = serial_to_int(serial_msg.substr(13, 4));
    int16_t axis5 = serial_to_int(serial_msg.substr(17, 4));
    int16_t axis6 = serial_to_int(serial_msg.substr(21, 4));

    double axis1_position = rover::map((double)axis1, 0, 4096, 3.14, -3.14);
    double axis2_position = rover::map((double)axis2, 0, 1024+512, -1.57, 1.57/2);
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    double axis3_position = rover::map((double)axis3, 512, 1024+512, -1.57/2, 1.57/2);
    double axis4_position = rover::map((double)axis4, 0, 9999, 6.28, -6.28);
    double axis5_position = rover::map((double)axis5, 0, 9999, -1.57, 1.57);
    double axis6_position = rover::map((double)axis6, 0, 9999, -6.28, 6.28);
    joint_position_[0] = axis1_position;
    joint_position_[1] = axis2_position;
    joint_position_[2] = axis3_position;
    joint_position_[3] = axis4_position;
    joint_position_[4] = axis5_position;
    joint_position_[5] = axis6_position;
  }
} //namespace arm22