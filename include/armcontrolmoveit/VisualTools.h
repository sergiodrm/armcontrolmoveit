/*
VisualTools.h
*/

#ifndef VISUALTOOLS_H_
#define VISUALTOOLS_H_

#include <ros/ros.h>
#include <armcontrolmoveit/ArmControl.h>
#include <armcontrolmoveit/Door.h>
#include <armcontrolmoveit/UsefulFunctions.h>
#include <math.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#define TEXT_HEIGHT 1.5

class ArmControl;
class Door;

class VisualTools
{
    public:
        VisualTools();
        VisualTools(const std::string &reference_frame);
        virtual ~VisualTools();

        moveit_visual_tools::MoveItVisualTools* getPtrVisualTools();
        void prompt(const std::string &str);

        void drawTrajectory(std::vector<geometry_msgs::Pose> wp, ArmControl &arm);
        void drawDoor(Door &door);
        void deleteAllMarkers();

    private:
        moveit_visual_tools::MoveItVisualTools *ptr_visual_tools;
        Eigen::Affine3d text_pose;

        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        
};

#endif