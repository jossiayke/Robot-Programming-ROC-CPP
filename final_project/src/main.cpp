#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include "../include/explorer.h"
#include <array>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  bool explorer_goal_sent = false; //do not need this
  bool follower_goal_sent = false; //do not need this
  bool follower_start = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  Explorer explorer(&nh, "robot_name");

  std::string robot_name;
  if (nh.hasParam("robot_name"))
  {
    nh.getParam("robot_name", robot_name);
    ROS_INFO_STREAM("robot name: " << robot_name);
  }

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true); // rostopic echo grep explorer ;* to see this
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  std::array<std::pair<double, double>, 4> targets;
  std::array<XmlRpc::XmlRpcValue, 4> target_loc;
  nh.getParam("/aruco_lookup_locations/target_1", target_loc[0]);
  nh.getParam("/aruco_lookup_locations/target_2", target_loc[1]);
  nh.getParam("/aruco_lookup_locations/target_3", target_loc[2]);
  nh.getParam("/aruco_lookup_locations/target_4", target_loc[3]);

  for (int32_t i = 0; i < target_loc.size(); ++i)
  {
    explorer.set_target_loc(&target_loc[i], i);
    targets.at(i).first = static_cast<double>(target_loc[i][0]);
    targets.at(i).second = static_cast<double>(target_loc[i][1]);
  }

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  //Build goal for explorer
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = targets.at(0).first;  //7.710214;//
  explorer_goal.target_pose.pose.position.y = targets.at(0).second; //-1.716889;//
  explorer_goal.target_pose.pose.orientation.w = 1.0;

  // //Build goal for follower
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();

  ROS_INFO("Sending goal");
  explorer_client.sendGoal(explorer_goal);

  ROS_INFO("Sending goal");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  // Create local array for storing marker location for follower
  std::array<std::pair<double, double>, 5> follow_path;

  // Initialize indx counter
  int indx = 0;
  while (ros::ok())
  {
    if (!follower_start)
    {
      if (!explorer_goal_sent)
      {
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal); //this should be sent only once
        explorer_goal_sent = true;
      }
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, robot reached goal");
        ROS_INFO_STREAM("idx: " << indx);
        if (indx < 4)
        {
          ROS_INFO("Rotation Began");
          /** Robot should start rotating here **/
          explorer.m_rotate(0, 0.2);
        }
        /** Capture Aruco Marker Id **/
        if (explorer.m_aruco_found)
        {
          ROS_INFO("ARUCO FOUND");
          explorer.m_listen(tfBuffer); // Retrieve marker_frame transforms

          if (explorer.m_transform_found)
          { // Check transform was actually recieved in listener
            indx++;
            explorer.m_rotate(0, 0); // Stop rotating robot
            if (indx < 4)
            {
              explorer_goal.target_pose.pose.position.x = targets.at(indx).first;  //7.710214;//
              explorer_goal.target_pose.pose.position.y = targets.at(indx).second; //-1.716889;//
              explorer_goal_sent = false;
              explorer.m_transform_found = false; // Reset flag
              explorer.m_aruco_found = false;     // Otherwise, keeps creating broadcasters thinking marker is detected
            }
            else if (indx == 4)
            {
              explorer_goal.target_pose.pose.position.x = -4;  //targets.at(0).first;//7.710214;//
              explorer_goal.target_pose.pose.position.y = 2.5; //
              explorer_goal_sent = false;
              explorer.m_broadcast_cancel = true;
            }
            else
            {
              // Final Follower Path
              for (int i = 0; i < 4; ++i)
              { // Store follow path into local array
                follow_path.at(i).first = explorer.get_marker_x(i);
                follow_path.at(i).second = explorer.get_marker_y(i);
              }

              // Add final target of going back to beginning
              follow_path.at(4).first = -4.0;
              follow_path.at(4).second = 3.5;

              ROS_INFO_STREAM("follow_path 1: " << follow_path[0].first << " " << follow_path[0].second);
              ROS_INFO_STREAM("follow_path 2: " << follow_path[1].first << " " << follow_path[1].second);
              ROS_INFO_STREAM("follow_path 3: " << follow_path[2].first << " " << follow_path[2].second);
              ROS_INFO_STREAM("follow_path 4: " << follow_path[3].first << " " << follow_path[3].second);
              ROS_INFO_STREAM("follow_path 5: " << follow_path[4].first << " " << follow_path[4].second);

              // Shutdown explorer before follower starts
              explorer_client.cancelAllGoals();
              explorer.sub_shutdown();
              ROS_INFO("Explorer Shutting Down");

              // Set follower to true
              follower_start = true;
              indx = 0;
            }
          }
        }
      }
    }
    else
    {

      if (indx == 0)
      {
        follower_goal.target_pose.pose.position.x = follow_path.at(0).first;  //-0.289296;//
        follower_goal.target_pose.pose.position.y = follow_path.at(0).second; //-1.282680;//
        follower_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal for follower");
        follower_client.sendGoal(follower_goal); //this should be sent only once
        follower_client.waitForResult();         //this should be sent only once
        follower_goal_sent = true;
      }
      if (!follower_goal_sent)
      {

        ROS_INFO_STREAM("follow_path: " << follow_path[indx].first << " " << follow_path[indx].second);

        follower_goal.target_pose.pose.position.x = follow_path[indx].first;
        follower_goal.target_pose.pose.position.y = follow_path[indx].second;

        ROS_INFO("Sending goal for follower");
        follower_client.sendGoal(follower_goal); //this should be sent only once
        follower_goal_sent = true;
      }

      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, robot reached goal");

        indx++;
        if (indx < 4)
        {
          follower_goal_sent = false;
        }
        else if (indx == 4)
        {
          ROS_INFO("Follower Back to Beginning");
          follower_goal_sent = false;
        }
        else
        {
          ROS_INFO("DONE! GOODBYE.");
          ros::shutdown();
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}