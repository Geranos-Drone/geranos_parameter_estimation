#include "ros/ros.h"

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>


int main(int argc, char **argv)
{

  // ros node
  ros::init(argc, argv, "test_client_node");
  ros::NodeHandle n;

  // client to set and reset q_mass
  ros::ServiceClient client_set = n.serviceClient<std_srvs::Empty>("set_mass_q_small");
  ros::ServiceClient client_reset = n.serviceClient<std_srvs::Empty>("set_mass_q_big");

  // empty service
  std_srvs::Empty srv;

  std::cout << "time before: " << ros::Time::now() << std::endl;

  // set q mass = 0
  client_set.call(srv);

  // wait 20 seconds (or whatever)
  ros::Duration d = ros::Duration(20, 0);
  d.sleep();

  /*here controller reads mass*/

  // set q mass >= 0.05
  client_reset.call(srv);

  std::cout << "time after: " << ros::Time::now().toSec() << std::endl;

  return 0;
}