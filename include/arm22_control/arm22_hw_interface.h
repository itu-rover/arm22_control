/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 *
 */

#ifndef ARM22_HW_INTERFACE_H
#define ARM22_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <serial/serial.h>
#include <thread>

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
    void feedback(std::string serial_msg);
    void read_loop();
    ros::Rate* ptr_write_rate;
    serial::Serial *serial_;
    std::string port;
    int baudrate;
    int read_rate;
    int write_rate;
    uint64_t ms_last;

    std::thread rx_thread;

    
  };
}

#endif
