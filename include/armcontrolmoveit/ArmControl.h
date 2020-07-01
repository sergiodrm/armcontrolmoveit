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
#include <queue>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <armcontrolmoveit/PoseArrayStamped.h>
#include <armcontrolmoveit/ChangeTarget.h>
#include <armcontrolmoveit/ExecuteTarget.h>
#include <armcontrolmoveit/DemoPrecision.h>
#include <armcontrolmoveit/HomeService.h>
#include <armcontrolmoveit/PlanTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <armcontrolmoveit/VisualTools.h>
#include <armcontrolmoveit/UsefulFunctions.h>


enum axis_enum {X, Y, Z, BD, BI, AD, AI};
enum type {articular, cartesian};
enum errorCodeTrajectory {SUCCESS, JOINT_DIFF, PLAN_ERROR};
const int MAX_QUEUE_SIZE = 2;
const float MAX_DIFF_JOINTS_VALUES = 0.01;

class VisualTools;

class ArmControl
{
public:

    /*
    * Constructor & Destructor 
    */
    ArmControl();
    ArmControl(char* ns);
    virtual ~ArmControl();
    
    /*
    * Methods to move & update position
    */
    bool move_to_point(const geometry_msgs::Pose &target);
    const float set_trajectory(const std::vector<geometry_msgs::Pose> &wp, float eef_step, float jump_threshold);
    void updateHome();
    int execute();
    void publishCartesianStates(const sensor_msgs::JointState &joint_msg);
    void publishCartesianPlanTrajectory();
    void publishJointPlanTrajectory();
    int checkPrintJointStates();

    /*
    * Services callbacks
    */
    bool changeTarget(armcontrolmoveit::ChangeTargetRequest &req, armcontrolmoveit::ChangeTargetResponse &res);
    bool planTrajectory(armcontrolmoveit::PlanTrajectoryRequest & req, armcontrolmoveit::PlanTrajectoryResponse &res);
    bool executeTarget(armcontrolmoveit::ExecuteTargetRequest &req, armcontrolmoveit::ExecuteTargetResponse &res);
    bool demoPrecision(armcontrolmoveit::DemoPrecisionRequest &req, armcontrolmoveit::DemoPrecisionResponse &res);
    bool homeService(armcontrolmoveit::HomeServiceRequest &req, armcontrolmoveit::HomeServiceResponse &res);

    /*
    * Topics callback
    */
    void jointStatesCallback(const sensor_msgs::JointState msg);

    /*
    * Get & Set
    */
    moveit::planning_interface::MoveGroupInterface::Plan getPlan();
    const moveit::core::JointModelGroup* getJointModelGroup();
private:
    ros::NodeHandle nh;
    ros::Rate *ptr_rate;
    armcontrolmoveit::PoseArrayStamped cartesianPlan;
    ros::Publisher pub_cartesian_plan, pub_cartesian_plan_NotArray;
    ros::Publisher pub_joint_plan;
    ros::Publisher pub_cartesian_states;
    std::pair<robot_model::RobotModelPtr, robot_state::RobotState*> kinematic_plan;
    std::pair<robot_model::RobotModelPtr, robot_state::RobotState*> kinematic_joint;
    std::queue<sensor_msgs::JointState> jointStatesMsgs;
    std::string planning_group;
    std::string rosNamespace;
    geometry_msgs::Pose home;
    moveit::planning_interface::MoveGroupInterface *ptr_move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    VisualTools *vstool;
};


#endif