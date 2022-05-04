/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 *
 */

#include <arm22_control/arm22_hw_interface.h>
#include <rover_utils/math_helpers.h>
#include <string>

const std::string DEBUG_NAME = "hw_interface";

bool is_number(std::string str){
  for(int i = 0; i < str.length(); i++){
    if(str[i] >= '0' && str[i] <= '9') continue;
    else return false;
  }
  return true;
}
namespace arm22
{
  arm22HWInterface::arm22HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    // Read settings.yaml
    nh.param<std::string>("/serial/port", this->port, "/please_fill/settings.yaml");
    nh.param("/serial/baudrate", this->baudrate, 1);
    nh.param("/serial/encoder_delta_threshold", this->encoder_delta_threshold, 0.1);

    allow_write_service_ = nh.advertiseService("/hardware_interface/allow_write", &arm22HWInterface::allow_write, this);

    // Initiate serial connection
    try
    {
      ROS_INFO_NAMED(DEBUG_NAME, "Trying to connect port: %s, baudrate: %d", this->port.c_str(), this->baudrate);
      serial_ = new serial::Serial(this->port, this->baudrate, serial::Timeout::simpleTimeout(200));
      if (serial_->isOpen())
      {
        ROS_INFO_NAMED(DEBUG_NAME, "Succesfully opened the serial port.");
      }
      else
      {
        ROS_ERROR_NAMED(DEBUG_NAME, "Failed to open the serial port.");
        ros::shutdown();
      }
    }
    catch (serial::IOException &e)
    {
      switch (e.getErrorNumber())
      {
      case 2:
        ROS_ERROR_NAMED(DEBUG_NAME, "No such file as '%s', TIP: check if the serial cable is connected.", this->port.c_str());
        break;
      case 13:
        ROS_ERROR_NAMED(DEBUG_NAME, "Failed to initiate serial, TIP: Check permissions.");
        break;
      default:
        ROS_ERROR_NAMED(DEBUG_NAME, "Failed to initiate serial. Full Error: %s", e.what());
        break;
      }

      ROS_ERROR_NAMED(DEBUG_NAME, "Exception while trying to initiate serial, SHUTTING DOWN!");
      ros::shutdown();
    }
  }

  bool arm22::arm22HWInterface::allow_write(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res){
    ALLOW_WRITE = req.data;
    res.success = true;
    res.message = (ALLOW_WRITE ? "WRITING ALLOWED" : "WRITING DISALLOWED");
    return true;
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

    std::string result = serial_->readline(27, "B");

    ROS_INFO("Got encoder message: %s, length: %ld", result.c_str(), result.size());
    feedback(result);


    comm_check_bit ^= 1;
    std::string msg_to_send = "";
    
    if (!ALLOW_WRITE)
    {
      joint_position_command_[0] = joint_position_[0];
      joint_position_command_[1] = joint_position_[1];
      joint_position_command_[2] = joint_position_[2];
      joint_position_command_[3] = joint_position_[3];
      joint_position_command_[4] = joint_position_[4];
      joint_position_command_[5] = joint_position_[5];
    }

    msg_to_send += "S";
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[0], 3.14, -3.14, 0, 4096), 0, 4096));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[1],-1.57, 1.57/2, 0, 1024 + 512), 256, 1024+512));
    // where 1.0732421875 = 3.14 * 700/2048  pi = 2048
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[2], -1.57, 1.57/2, 0, 1024+512), 350, 1024+512));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[3], 6.28, -6.28, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[4], -1.57, 1.57, 0, 9999), 0, 9999));
    msg_to_send += to_serial(rover::clamp(rover::map(joint_position_command_[5], -6.28, 6.28, 0, 9999), 0, 9999));
    msg_to_send += (comm_check_bit ? "1" : "0");
    msg_to_send += "F";
    
    ROS_INFO("%sSerial message: [%s]", (ALLOW_WRITE ? "\033[1;32m" : "\033[1;31m"),msg_to_send.c_str());

    if(ALLOW_WRITE){
      serial_->write(msg_to_send);
    }
  }

  void arm22HWInterface::feedback(std::string serial_msg)
  {
    /* Assuming serial_msg has the following pattern:

      A AXIS AXIS AXIS AXIS AXIS AXIS B
      total length = 26

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
      double axis3_position = rover::map((double)axis3, 0, 1024+512, -1.57, 1.57/2);
      double axis4_position = rover::map((double)axis4, 0, 9999, 6.28, -6.28);
      double axis5_position = rover::map((double)axis5, 0, 9999, -1.57, 1.57);
      double axis6_position = rover::map((double)axis6, 0, 9999, -6.28, 6.28);
      ROS_INFO("%d", ALLOW_WRITE);
      if (ALLOW_WRITE && (
          fabs(axis1_position - joint_position_[0]) > encoder_delta_threshold ||
          fabs(axis2_position - joint_position_[1]) > encoder_delta_threshold ||
          fabs(axis3_position - joint_position_[2]) > encoder_delta_threshold ||
          fabs(axis4_position - joint_position_[3]) > encoder_delta_threshold ||
          fabs(axis5_position - joint_position_[4]) > encoder_delta_threshold ||
          fabs(axis6_position - joint_position_[5]) > encoder_delta_threshold)){
            encoder_error_count++;
            ROS_ERROR("ERROR COUNT: %d | Encoder got a message with unexpected change. [%s] %lf %lf %lf %lf %lf %lf", encoder_error_count, serial_msg.c_str(), axis1_position, axis2_position, axis3_position, axis4_position, axis5_position, axis6_position);
            while(encoder_error_count > 5){
              ROS_ERROR("ERROR COUNT: %d | Encoder got a message with unexpected change. [%s] %lf %lf %lf %lf %lf %lf", encoder_error_count, serial_msg.c_str(), axis1_position, axis2_position, axis3_position, axis4_position, axis5_position, axis6_position);
              serial_->write("S555555F");
              ros::Duration(5).sleep();
            }
        }
        else{
          encoder_error_count = 0;
          joint_position_[0] = axis1_position;
          joint_position_[1] = axis2_position;
          joint_position_[2] = axis3_position;
          joint_position_[3] = axis4_position;
          joint_position_[4] = axis5_position;
          joint_position_[5] = axis6_position;
        }
      }
    catch(std::invalid_argument &e){
      return;
    }
  }
}