/* Pick and place code inspired by moveit tutorials (Ioan Sucan, Ridhwan Luthra) */

#include <cstdlib>
// ROS
#include <ros/ros.h>
#include <stdio.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// services
#include "cpp_panda_pick_place/PickAndPlace.h"
#include "cpp_panda_pick_place/ResetScene.h"


// ------------------- functions called ----------------------
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pickObject(moveit::planning_interface::MoveGroupInterface& move_group, std::string cube_name)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

// moveit_msgs::RobotTrajectory trajectory;
// const double jump_threshold = 0.5; 
// const double eef_step = 0.01;
// move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;

  float posX, posY;
  if (cube_name == "red_cube")
  {
    posX = 0.6;
    posY = 0.0;
  }else if (cube_name == "green_cube")
  {
    posX = -0.3;
    posY = -0.4;
  }else if (cube_name == "blue_cube")
  {
    posX = 0.6;
    posY = -0.4;
  }

  // orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  orientation.setRPY(-M_PI, 0, -M_PI / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = posX;
  grasps[0].grasp_pose.pose.position.y = posY;
  grasps[0].grasp_pose.pose.position.z = 0.035 + 0.115;


  // Setting pre-grasp approach w.r.t. frame_id
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  // grasps[0].pre_grasp_approach.direction.vector.z = 1.0;

  // grasps[0].pre_grasp_approach.min_distance = 0.095;
  // grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].pre_grasp_approach.min_distance = 0.005;
  grasps[0].pre_grasp_approach.desired_distance = 0.7;

  // Setting post-grasp retreat w.r.t frame_id
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0005;
  grasps[0].post_grasp_retreat.desired_distance = 0.825;

  // grasps[0].post_grasp_retreat.min_distance = 0.1;
  // grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // set tolerances
  move_group.setGoalJointTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setGoalPositionTolerance(0.01);

  // Setting eef posture before and during grasp
  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  // Set ground as support surface
  move_group.setSupportSurfaceName("ground_plane");
  
  // Call pick to pick up the object using the grasps given
  move_group.pick(cube_name, grasps);

}

void placeObject(moveit::planning_interface::MoveGroupInterface& group, std::string cube_name)
{
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach w.r.t frame_id
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat w.r.t to frame_id
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  group.setGoalJointTolerance(0.005);
  group.setGoalOrientationTolerance(0.04);
  group.setGoalPositionTolerance(0.04);

  // Setting posture of eef after placing object
  openGripper(place_location[0].post_place_posture);
  // Set support surface as ground_plane
  group.setSupportSurfaceName("ground_plane");
  // Call place to place the object using the place locations given.
  group.place(cube_name, place_location);
}

void buildCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string cube_name)
{
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // // Add ground plane
  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "ground_plane";

  // /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 2.5;
  collision_objects[0].primitives[0].dimensions[1] = 2.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.2;

  // /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = -0.105;
  // // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].id = cube_name;

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.07;
  collision_objects[1].primitives[0].dimensions[1] = 0.07;
  collision_objects[1].primitives[0].dimensions[2] = 0.07;

  float posX, posY;
  if (cube_name == "red_cube")
  {
    posX = 0.6;
    posY = 0.0;
  }else if (cube_name == "green_cube")
  {
    posX = -0.3;
    posY = -0.4;
  }else if (cube_name == "blue_cube")
  {
    posX = 0.6;
    posY = -0.4;
  }

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = posX;
  collision_objects[1].primitive_poses[0].position.y = posY;
  collision_objects[1].primitive_poses[0].position.z = 0.035;
  // END_SUB_TUTORIAL

  // collision_objects[1].operation = collision_objects[1].ADD;

  //   // Define the object that we will be manipulating
  // collision_objects[2].header.frame_id = "panda_link0";
  // collision_objects[2].id = "green_cube";

  // /* Define the primitive and its dimensions. */
  // collision_objects[2].primitives.resize(1);
  // collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
  // collision_objects[2].primitives[0].dimensions.resize(3);
  // collision_objects[2].primitives[0].dimensions[0] = 0.07;
  // collision_objects[2].primitives[0].dimensions[1] = 0.07;
  // collision_objects[2].primitives[0].dimensions[2] = 0.07;

  // /* Define the pose of the object. */
  // collision_objects[2].primitive_poses.resize(1);
  // collision_objects[2].primitive_poses[0].position.x = 0.05;
  // collision_objects[2].primitive_poses[0].position.y = 0.4;
  // collision_objects[2].primitive_poses[0].position.z = 0.035;
  // // END_SUB_TUTORIAL

  // collision_objects[2].operation = collision_objects[2].ADD;


  // // Define the object that we will be manipulating
  // collision_objects[3].header.frame_id = "panda_link0";
  // collision_objects[3].id = "blue_cube";

  // /* Define the primitive and its dimensions. */
  // collision_objects[3].primitives.resize(1);
  // collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
  // collision_objects[3].primitives[0].dimensions.resize(3);
  // collision_objects[3].primitives[0].dimensions[0] = 0.07;
  // collision_objects[3].primitives[0].dimensions[1] = 0.07;
  // collision_objects[3].primitives[0].dimensions[2] = 0.07;

  // /* Define the pose of the object. */
  // collision_objects[3].primitive_poses.resize(1);
  // collision_objects[3].primitive_poses[0].position.x = 0.6;
  // collision_objects[3].primitive_poses[0].position.y = -0.4;
  // collision_objects[3].primitive_poses[0].position.z = 0.035;
  // // END_SUB_TUTORIAL

  // collision_objects[3].operation = collision_objects[3].ADD;


  planning_scene_interface.applyCollisionObjects(collision_objects);
  // added for debug
  ros::Duration(2.0).sleep();
  planning_scene_interface.getKnownObjectNames();
}

void pickAndPlaceObject(moveit::planning_interface::MoveGroupInterface& group, std::string cube_name)
{
  ros::WallDuration(3.0).sleep();
  pickObject(group,cube_name);
  ros::WallDuration(3.0).sleep();
  placeObject(group,cube_name);
}


// ------------------- service node ----------------------
bool pickAndPlaceCube(cpp_panda_pick_place::PickAndPlace::Request &req,
                            cpp_panda_pick_place::PickAndPlace::Response &res)
{
  ROS_INFO("----------- Going to pick cube of colour : %s .", req.colour.c_str());
  std::string cube_name = req.colour.c_str();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);
  buildCollisionObjects(planning_scene_interface, cube_name);
  pickAndPlaceObject(group,cube_name);

  res.status = "picked & placed";
  ROS_INFO("----------- Cube of colour : %s is %s", req.colour.c_str(), res.status.c_str());
  return true;
}

// ------------------- main function ----------------------
int main(int argc, char** argv)
{
  // initialize service node
  ros::init(argc, argv, "pick_place_cube_client");
  if (argc != 2)
  {
   ROS_INFO("usage: pick_place_cube_client COLOUR");
   return 1;
  }

  // build nodehandle and asyncspinner
  // ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // create pick place server
  ros::ServiceServer service = nh.advertiseService("pick_place_cube_service", pickAndPlaceCube);
  ROS_INFO("----------- pick_place_cube_service is READY ");
  // ros::spin();

// IF FOLLOWING COMMENTED CALL : rosservice call /panda_pick_place_cube_service '{colour: "red_cube"}'
  // // create pick place client
  // ros::ServiceClient client = nh.serviceClient<cpp_panda_pick_place::PickAndPlace>("pick_place_cube_service");
  // cpp_panda_pick_place::PickAndPlace srv;
  // srv.request.colour = atoll(argv[1]);
  // if (client.call(srv))
  // {
  //   ROS_INFO("status: %s .", srv.response.status.c_str());
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service pick_place_cube_service");
  //   return 1;
  // }

  ros::waitForShutdown();

  return 0;
}