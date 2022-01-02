/**
 * @author B. Burak Payzun
 * @date 2022-01-02
 * 
 */

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <arm21_control/arm21_hw_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm21_hw_interface");
  ros::NodeHandle nh;

  // run the ROS loop in a separate thread
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // create our own hardware interface and initialize it
  boost::shared_ptr<arm21::Arm21HWInterface> arm21_hw_interface(new arm21::Arm21HWInterface(nh));
  arm21_hw_interface->init();

  // start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, arm21_hw_interface);
  control_loop.run();

  return 0;
}
