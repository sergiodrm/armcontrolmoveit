/*
ArmControl.h
Header de la clase ArmControl usada para comunicarse directamente 
con MoveIt! y leer o ejecutar las trayectorias pertinentes.

Autor: sergiodrm
Fecha: Marzo 2020
*/

#ifndef ARMCONTROL_H_
#define ARMCONTROL_H_

#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <armcontrolmoveit/ChangeTarget.h>
#include <armcontrolmoveit/ExecuteTarget.h>
#include <armcontrolmoveit/DemoPrecision.h>
#include <armcontrolmoveit/HomeService.h>
#include <armcontrolmoveit/PlanTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <armcontrolmoveit/VisualTools.h>

enum axis_enum {X, Y, Z, BD, BI, AD, AI};
enum type {articular, cartesian};

class VisualTools;

class ArmControl
{
public:

    /*
    * Constructor & Destructor 
    */
    ArmControl();
    virtual ~ArmControl();
    
    /*
    * Methods to move & update position
    */
    bool move_to_point(const geometry_msgs::Pose &target);
    const float set_trajectory(const std::vector<geometry_msgs::Pose> &wp, float eef_step, float jump_threshold);
    void updateHome();
    void execute();

    /*
    * Services callbacks
    */
    bool change_target(armcontrolmoveit::ChangeTargetRequest &req, armcontrolmoveit::ChangeTargetResponse &res);
    bool plan_trajectory(armcontrolmoveit::PlanTrajectoryRequest & req, armcontrolmoveit::PlanTrajectoryResponse &res);
    bool execute_target(armcontrolmoveit::ExecuteTargetRequest &req, armcontrolmoveit::ExecuteTargetResponse &res);
    bool demo_precision(armcontrolmoveit::DemoPrecisionRequest &req, armcontrolmoveit::DemoPrecisionResponse &res);
    bool home_service(armcontrolmoveit::HomeServiceRequest &req, armcontrolmoveit::HomeServiceResponse &res);

    /*
    * Get & Set
    */
    moveit::planning_interface::MoveGroupInterface::Plan getPlan();
    const moveit::core::JointModelGroup* getJointModelGroup();
private:
    ros::NodeHandle nh;
    ros::Rate *ptr_rate;
    std::string planning_group;
    geometry_msgs::Pose home;
    moveit::planning_interface::MoveGroupInterface *ptr_move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    VisualTools *vstool;
};


#endif