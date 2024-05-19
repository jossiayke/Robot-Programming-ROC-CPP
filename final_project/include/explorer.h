/**
 * @file explorer.h
 * @author Hae Lee Kim, Yoseph Kebedee, Mohammed Baaqail
 * @brief The file contains the Explorer class
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
/*! \mainpage 
*
*This project is inspired by the challenge of autonomous 
*roboticsation to the second robot "follower". The explorer
* will head back to base location.
*
*- \subpage The "follower" robot is going to follow the path generated
* based on the information given by "explorer". The follower is responsible 
* for rescuing the victims in the order given by "explorer". After rescuing 
* all the victims, the "follower" will head back to base location. 
* 
*- \subpage Per this proj for Urban Search and Rescue (US&R).
*In US&R after a disaster occurs, a robot is used too explore an 
*unknown environment, such as a partially collapsed building, 
*and locates trapped or unconscious human victims of the disaster.

*- \subpage In this project, we will be simulating this scenario 
*using 2 Turtlebot3 Waffale model robots. 
*
*
*- \subpage The first robot is called "expolrer" which has the responsibility
* of discovering the map and locate the AruCo markers (victims). Then send the 
* the location informect, there will be 4 AruCo markers distributed across
* the map, each one will have a unique ID and fiducial square barcode. 
*
*- Professor's refrence github code <a href="https://github.com/zeidk/final_project.git">click here</a>.


*/

#ifndef EXPLORER_H
#define EXPLORER_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <ros/ros.h>
#include <iostream>
#include <array>
#include <utility>
#include <tf/transform_datatypes.h> // to manipulate quaternions
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/**
 * @brief This class used to generate path and detect ArUco markers
 * 
 * 
 */
class Explorer
{
public:
    /**
     * @brief Construct a new Explorer object
     * 
     * @param nodehandle a roscpp interface to interact with publisher and subscribers
     * @param robot_name string for the robot name 
     */
    Explorer(ros::NodeHandle *nodehandle, const std::string &robot_name);

    bool m_aruco_found; // flag to check if arcuo marker is found

    const double get_current_x() // x coordinate of the current location of robot
    {
        return m_location.first;
    }

    const double get_current_y() // y coordinate of the current location of robot
    {
        return m_location.second;
    }

    const double get_marker_x(int ind) // x coordinate of the ArUco marker location
    {
        return m_transforms[ind].first;
    }

    const double get_marker_y(int ind) // y coordinate of the ArUco marker location
    {
        return m_transforms[ind].second;
    }

    /**
     * @brief Set the target location for the object
     * 
     * 
     * @param target the value of the target location
     * @param i the id of the target 
     */
    void set_target_loc(XmlRpc::XmlRpcValue *target, int i)
    {
        m_target_loc[i] = target;
    }

    /**
     * @brief a funtion to update the boolean state when goal is reached
     * 
     * @param val returns whether goal is reached or not
     */
    void set_goal_reached(bool val)
    {
        m_goal_reached = val;
    }

    /**
     * @brief a function to control the rotation of the robot
     * 
     * @param linear linear velocity parameter
     * @param angular angular velocity parameter
     */
    void m_rotate(double linear, double angular);

    /**
     * @brief a buffer to present an interface to interact with tf_frames
     * 
     * @param tfBuffer recieves transformation messages for the listener
     */
    void m_listen(tf2_ros::Buffer &tfBuffer);

    /**
     * @brief a funtion to unsubscribe the callback function associated with it
     * 
     */
    void sub_shutdown()
    {
        m_fid_subscriber.shutdown();
    }

    /**
     * @brief a function to detect ArUco markers for the broadcaster
     * 
     * @param fid_msg recieve messages for the broadcaster transformation
     */
    void m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg);

    /**
     * @brief a function to get the pose of the robot
     * 
     * @param amcl_msg a parameter to help robot with localization
     */
    void m_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg);

    /**
     * @brief Initializes the neccessary subscriber's parameters for the robot such as pose
     * 
     */
    void m_initialize_subscribers();

    /**
     * @brief Initializes the neccessary publisher's parameters for the robot such as velocity
     * 
     */
    void m_initialize_publishers();

    bool m_transform_found = false;
    bool m_broadcast_cancel; // Stop broadcasting after explorer back to original spot

private:
    ros::NodeHandle m_nh; // we will need this, to pass between main() and constructor

    // Publisher & Subscriber objects
    ros::Subscriber m_pose_subscriber;
    ros::Subscriber m_fid_subscriber;
    ros::Publisher m_rotate_publisher;

    std::string m_robot_name;
    bool m_goal_reached = false;
    bool m_mark_true = false;
    std::pair<double, double> m_location;
    std::pair<double, double> m_tolerance = std::make_pair(0, 0);
    std::array<std::pair<double, double>, 4> m_aruco_markers;
    std::array<std::pair<double, double>, 4> m_transforms; // store transforms of marker from listerner
    std::array<XmlRpc::XmlRpcValue, 4> m_target_loc;

    int m_fiducial_id;
    geometry_msgs::Quaternion m_orientation;
};
#endif