#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/////////////////////////////////////////////////////

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  //auto pick_place_node = std::make_shared<pick_place_node>(options);
//////


 auto const node = std::make_shared<rclcpp::Node>(
    "pick_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("pick_move");
  
  //static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_move");
  

// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");
auto move_group_hand = MoveGroupInterface(node, "panda_hand");

// Open gripper
  std::map<std::string, double> joint_values;
  joint_values["panda_finger_joint1"] = 0.04; 
  joint_values["panda_finger_joint2"] = 0.04; 
  
  move_group_hand.setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
   bool hand_success = static_cast<bool>(move_group_hand.plan(hand_plan));
  
  if (hand_success) {
    move_group_hand.execute(hand_plan);
  }


tf2::Quaternion q;
q.setRPY(3, 0, 0);


// Set a target Pose
auto const target_pose = [q]{
  geometry_msgs::msg::Pose msg;
  msg.orientation = tf2::toMsg(q);
  msg.position.x = 0.1;
  msg.position.y = 0.0;
  msg.position.z = 0.9;
 
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
   std::map<std::string, double> joint_values;
   // Close gripper
   //joint_values["panda_finger_joint1"] = 0.04; 
   //joint_values["panda_finger_joint2"] = 0.04; 
   move_group_hand.setJointValueTarget(joint_values);
   
   moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
   bool hand_success = static_cast<bool>(move_group_hand.plan(hand_plan));
  
  if (hand_success) {
    move_group_hand.execute(hand_plan);
  }
   
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
};


///////


tf2::Quaternion q1;
q1.setRPY(3, 0, 0);

// Continue pickup after opening gripper (and a little extra movement)
auto const target_pose1 = [q1]{
  geometry_msgs::msg::Pose msg;
  msg.orientation = tf2::toMsg(q1);
  msg.position.x = 0.7;
  msg.position.y = 0.0;
  msg.position.z = 0.4;
 
  return msg;
}();
move_group_interface.setPoseTarget(target_pose1);

// Create a plan to that target pose
auto const [success1, plan1] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan1);
   std::map<std::string, double> joint_values;
   // Close gripper
   joint_values["panda_finger_joint1"] = 0.0; 
   joint_values["panda_finger_joint2"] = 0.0; 
   move_group_hand.setJointValueTarget(joint_values);
   
   moveit::planning_interface::MoveGroupInterface::Plan hand_plan1;
   bool hand_success = static_cast<bool>(move_group_hand.plan(hand_plan1));
  
  if (hand_success) {
    move_group_hand.execute(hand_plan1);
  }
   
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
};



// Place object

tf2::Quaternion q2;
q2.setRPY(3, 0, 0);


auto const target_pose2 = [q2]{
  geometry_msgs::msg::Pose msg;
  msg.orientation = tf2::toMsg(q2);
  msg.position.x = -0.5;
  msg.position.y = 0.5;
  msg.position.z = 0.4;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose2);

// Create a plan to that target pose
auto const [success2, plan2] = [&move_group_interface, &move_group_hand]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan2);
  std::map<std::string, double> joint_values;
  joint_values["panda_finger_joint1"] = 0.04;
  joint_values["panda_finger_joint2"] = 0.04; 
  move_group_hand.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan hand_plan2;
   bool hand_success = static_cast<bool>(move_group_hand.plan(hand_plan2));
  
  if (hand_success) {
    move_group_hand.execute(hand_plan2);
  }
  
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
};

// Return to home
tf2::Quaternion q3;
q3.setRPY(3, 0, 0);


auto const target_pose4 = [q3]{
  geometry_msgs::msg::Pose msg;
  msg.orientation = tf2::toMsg(q3);
  msg.position.x = 0.0;
  msg.position.y = 0.0;
  msg.position.z = 1.0;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose4);

// Create a plan to that target pose
auto const [success3, plan3] = [&move_group_interface, &move_group_hand]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan3);
  std::map<std::string, double> joint_values;
  joint_values["panda_finger_joint1"] = 0.04;
  joint_values["panda_finger_joint2"] = 0.04; 
  move_group_hand.setJointValueTarget(joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan hand_plan3;
   bool hand_success = static_cast<bool>(move_group_hand.plan(hand_plan3));
  
  if (hand_success) {
    move_group_hand.execute(hand_plan3);
  }
  
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
};



}
