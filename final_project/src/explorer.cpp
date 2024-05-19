#include "../include/explorer.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

Explorer::Explorer(ros::NodeHandle *nodehandle, const std::string &robot_name) : m_robot_name{robot_name},
                                                                                 m_nh{*nodehandle},
                                                                                 m_location{0, 0},
                                                                                 m_transform_found{false},
                                                                                 m_broadcast_cancel{false}
{
    m_initialize_subscribers();
    m_initialize_publishers();
}

void Explorer::m_initialize_publishers()
{
    ROS_INFO("Initializing Publishers");
    m_rotate_publisher = m_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);
}

void Explorer::m_initialize_subscribers()
{
    ROS_INFO("Initializing Subscribers");
    m_pose_subscriber = m_nh.subscribe("explorer/amcl_pose", 1000, &Explorer::m_pose_callback, this);
    m_fid_subscriber = m_nh.subscribe("/fiducial_transforms", 1000, &Explorer::m_fiducial_callback, this);
}

// Rotate robot
void Explorer::m_rotate(double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    m_rotate_publisher.publish(msg);
    m_goal_reached = true;
}

// Get pose of robot
void Explorer::m_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
    m_location.first = amcl_msg->pose.pose.position.x;
    m_location.second = amcl_msg->pose.pose.position.y;
    m_orientation = amcl_msg->pose.pose.orientation;
}

void Explorer::m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg)
{
    if (!fid_msg->transforms.empty() && !m_broadcast_cancel)
    { //check marker is detected and broadcast not canceled

        //broadcaster object
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        //broadcast the new frame to /tf Topic
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame";

        transformStamped.transform.translation.x = fid_msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = fid_msg->transforms[0].transform.translation.y;
        transformStamped.transform.translation.z = fid_msg->transforms[0].transform.translation.z;
        transformStamped.transform.rotation.x = fid_msg->transforms[0].transform.rotation.x;
        transformStamped.transform.rotation.y = fid_msg->transforms[0].transform.rotation.y;
        transformStamped.transform.rotation.z = fid_msg->transforms[0].transform.rotation.z;
        transformStamped.transform.rotation.w = fid_msg->transforms[0].transform.rotation.w;

        auto x_dis = abs(static_cast<double>(transformStamped.transform.translation.x));
        auto y_dis = abs(static_cast<double>(transformStamped.transform.translation.y));

        if (m_goal_reached && (x_dis < 2 && y_dis < 2))
        {

            m_fiducial_id = fid_msg->transforms[0].fiducial_id;

            br.sendTransform(transformStamped);
            ROS_INFO("Created Broadcaster");
            m_aruco_found = true;   // need to check we are at the goal before publishing transform
            m_goal_reached = false; // To only broadcast 1 time
        }
    }
}

void Explorer::m_listen(tf2_ros::Buffer &tfBuffer)
{
    //for listener
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));

        auto trans_x = static_cast<double>(transformStamped.transform.translation.x);
        auto trans_y = static_cast<double>(transformStamped.transform.translation.y);
        auto trans_z = transformStamped.transform.translation.z;

        double dis_x = trans_x - m_location.first;
        double dis_y = trans_y - m_location.second;

        if (dis_x > 0)
        {
            m_transforms.at(m_fiducial_id).first = trans_x - 0.4; //m_tolerance.first;
        }
        else if (dis_x <= 0)
        {
            m_transforms.at(m_fiducial_id).first = trans_x + 0.4; //m_tolerance.first;
        }

        if (dis_y > 0)
        {
            m_transforms.at(m_fiducial_id).second = trans_y - 0.4; //m_tolerance.second;
        }
        else if (dis_y <= 0)
        {
            m_transforms.at(m_fiducial_id).second = trans_y + 0.4; //m_tolerance.second;
        }

        m_transform_found = true; // set true when listener works

        ROS_INFO_STREAM("m_transforms: " << m_transforms.at(m_fiducial_id).first << " , "
                                         << m_transforms.at(m_fiducial_id).second);

        ROS_INFO_STREAM("fiducial_id: " << m_fiducial_id);

        ROS_INFO_STREAM("Position in map frame: ["
                        << trans_x << ","
                        << trans_y << ","
                        << trans_z << "]");
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}
