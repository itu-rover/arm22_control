/**
 * @author B. Burak Payzun
 * @date 2022-02-10
 *
 */

#ifndef ARM22_SIM_INTERFACE_H
#define ARM22_SIM_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <std_srvs/Trigger.h>

namespace arm22
{
  class arm22SimInterface : public ros_control_boilerplate::GenericHWInterface
  {
  public:
    arm22SimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

    virtual void init();
    virtual void read(ros::Duration &elapsed_time);
    virtual void write(ros::Duration &elapsed_time);

    virtual void enforceLimits(ros::Duration &period);

  protected:
    bool toggle_write(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    void feedback(std::string serial_msg);
    ros::ServiceClient _client;
    
    ros::ServiceServer write_toggle_service_;
  };
}

#endif
