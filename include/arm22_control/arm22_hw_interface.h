/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 *
 */

#ifndef ARM22_HW_INTERFACE_H
#define ARM22_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <serial/serial.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace arm22
{
  class arm22HWInterface : public ros_control_boilerplate::GenericHWInterface
  {
  public:
    arm22HWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

    virtual void init();
    virtual void read(ros::Duration &elapsed_time);
    virtual void write(ros::Duration &elapsed_time);

    virtual void enforceLimits(ros::Duration &period);

  protected:
    bool allow_write(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    void gripper_control_callback(const std_msgs::Float64::ConstPtr& msg);

    void feedback(std::string serial_msg);
    int encoder_error_count = 0;
  
    double current_gripper_pos = 0;

    bool ALLOW_WRITE = false;
    
    ros::Publisher gripper_state_publisher;
    ros::Publisher gripper_button_state_publisher;
    ros::Subscriber gripper_controller;
    ros::ServiceServer allow_write_service_;
    serial::Serial *serial_;
    std::string port;
    int baudrate;
    double encoder_delta_threshold;
  };
}

#endif
