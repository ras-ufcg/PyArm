#include <ros/ros.h>
#include "std_msgs/String.h"

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

/**
 * This program loads the models and make the kinematics computing
 */

void chatterCallback(const geometry_msgs::Pose& msg)  // Pesquisar mais sobre a recepção desse tipo
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // ROS Package details
  ros::init(argc, argv, "kinematics");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs/ByteMultiArray>("joints", 1000);  // Conferir esse tipo
  ros::Subscriber sub = n.subscribe("coordinates", 1000, chatterCallback); // Necessita de um spinner

  ros::AsyncSpinner spinner(1);
  spinner.start(); // Testar/pesquisar se isso resolve o subscriber

  // MoveIt initial configurations

  // Load the pyarm_description package with URDF and SRDF
  robot_model_loader::RobotModelLoader robot_model_loader("pyarm_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("pyarm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  ros::Rate loop_rate(10);

  // Get Joint Values
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Set an position
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  // Forward Kinematics
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8"); // Getting the position os 

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // Inverse Kinematics
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

  ros::shutdown();
  return 0;
}


