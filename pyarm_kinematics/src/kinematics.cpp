#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void callback(const geometry_msgs::Point& target)  // Pesquisar mais sobre a recepção desse tipo
{
  std::vector<double> joint_values;

  // Inverse Kinematics
  const geometry_msgs::Pose& end_effector_state;
  end_effector_state.position = target;
  end_effector_state.orientation.x = 1;
  end_effector_state.orientation.y = 0;
  end_effector_state.orientation.z = 0;
  end_effector_state.orientation.w = 0;

  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      pub.publish(joint_values)
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
}

int main(int argc, char **argv)
{
  // ROS Package details
  ros::init(argc, argv, "kinematics");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<joints>("joints", 1000);  // Conferir esse tipo
  ros::Subscriber sub = n.subscribe("coordinates", 1000, callback);

  // MoveIt initial configurations

  // Load the pyarm_description package with URDF and SRDF
  robot_model_loader::RobotModelLoader robot_model_loader("pyarm_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("pyarm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();;

  ros::spin();
  return 0;
}


