/**
 * @author B. Burak Payzun
 * @date 2022-01-01
 * 
 */

#ifndef ARM21_HW_INTERFACE_H
#define ARM21_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <serial/serial.h>

namespace arm21
{
class Arm21HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  Arm21HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
  
  virtual void init();
  virtual void read(ros::Duration& elapsed_time);
  virtual void write(ros::Duration& elapsed_time);

  virtual void enforceLimits(ros::Duration& period);

protected:
  void feedback(std::string serial_msg);

  serial::Serial* serial_;
};
}

#endif
