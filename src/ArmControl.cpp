/*
ArmControl.cpp

Autor: sergiodrm
Fecha: Marzo 2020
*/

#include <armcontrolmoveit/ArmControl.h>


/*
*           Constructor & Destructor
*/

ArmControl::ArmControl()
{
    ros::param::get("/move_group_config/planning_group", this->planning_group);
    this->ptr_move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
    ROS_INFO("Move_group connected with planning_group: %s", this->planning_group.c_str());
    // We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", this->ptr_move_group->getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("End effector link: %s", this->ptr_move_group->getEndEffectorLink().c_str());

    double x, y, z, roll, pitch, yaw;
    tf::Quaternion q;
    ros::param::get("/config_arm/home/x", x);
    ros::param::get("/config_arm/home/y", y);
    ros::param::get("/config_arm/home/z", z);
    ros::param::get("/config_arm/home/roll", roll);
    ros::param::get("/config_arm/home/pitch", pitch);
    ros::param::get("/config_arm/home/yaw", yaw);
    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    q.normalize();

    this->home.position.x = x;
    this->home.position.y = y;
    this->home.position.z = z;
    this->home.orientation.x = q.getX();
    this->home.orientation.y = q.getY();
    this->home.orientation.z = q.getZ();
    this->home.orientation.w = q.getW();
    this->vstool = new VisualTools(this->ptr_move_group->getPlanningFrame());
}

ArmControl::~ArmControl(){
    delete this->ptr_move_group;
    delete this->ptr_rate;
}

/*
*        Methods to move & update position  
*/

bool ArmControl::move_to_point(const geometry_msgs::Pose &target){
    this->ptr_move_group->setPoseTarget(target);
    
    bool success =  this->ptr_move_group->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    std::vector<geometry_msgs::Pose> wp;
    wp.push_back(target);
    
    this->vstool->drawTrajectory(wp, *this);
    return success;
}

const float ArmControl::set_trajectory(const std::vector<geometry_msgs::Pose> &wp, float eef_step, float jump_threshold)
{
    float fraction = this->ptr_move_group->computeCartesianPath(wp, eef_step, jump_threshold, this->my_plan.trajectory_);
    if (fraction <= 0)
    {
        ROS_ERROR("Error planning... (fraction = %f)", fraction);
    } else {
        ROS_INFO("Planning successful! (fraction = %f)", fraction);
        this->vstool->drawTrajectory(wp, *this);
    }
    return fraction;
}

void ArmControl::updateHome()
{
    double x, y, z, roll, pitch, yaw;
    tf::Quaternion q;
    ros::param::get("/config_arm/home/x", x);
    ros::param::get("/config_arm/home/y", y);
    ros::param::get("/config_arm/home/z", z);
    ros::param::get("/config_arm/home/roll", roll);
    ros::param::get("/config_arm/home/pitch", pitch);
    ros::param::get("/config_arm/home/yaw", yaw);
    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    q.normalize();

    this->home.position.x = x;
    this->home.position.y = y;
    this->home.position.z = z;
    this->home.orientation.x = q.getX();
    this->home.orientation.y = q.getY();
    this->home.orientation.z = q.getZ();
    this->home.orientation.w = q.getW();
}

void ArmControl::execute()
{
    this->ptr_move_group->execute(this->my_plan);
}


/*
* *********                     ************
*           Services Callbacks
* *********                     ************
*/

bool ArmControl::change_target(armcontrolmoveit::ChangeTargetRequest &req, armcontrolmoveit::ChangeTargetResponse &res)
{
    tf::Quaternion q;
    q.setRPY(req.roll*M_PI/180, req.pitch*M_PI/180, req.yaw*M_PI/180);
    q.normalize();

    geometry_msgs::Pose t;
    t.orientation.x = q.getX();
    t.orientation.y = q.getY();
    t.orientation.z = q.getZ();
    t.orientation.w = q.getW();
    t.position.x = req.x;
    t.position.y = req.y;
    t.position.z = req.z;
    
    ROS_INFO("Sending message...");
    res.success = this->move_to_point(t);
    ROS_INFO("Message received...");
    
    std::cout << "\033[34mTarget received: \033[0m[" << t.position.x << ", " << t.position.y << ", " << t.position.z << "] [";
    std::cout << req.roll << ", " << req.pitch << ", " << req.yaw << "] \033[0m\n";
    return true;
}

bool ArmControl::plan_trajectory(armcontrolmoveit::PlanTrajectoryRequest &req, armcontrolmoveit::PlanTrajectoryResponse &res)
{
    bool success = false;
    if (req.type == type::articular)
    {
        success = this->move_to_point(req.wp.at(req.wp.size()-1));
    } else if (req.type == type::cartesian)
    {
        if (req.eef_step > 0 && req.jump_threshold >= 0)
        {
            res.fraction = this->set_trajectory(req.wp, req.eef_step, req.jump_threshold);
        } else {
            float eef_step = 0.01, jump_threshold = 0;
            if (req.eef_step <= 0)
            {
                ROS_WARN("Step size not allowed (default = 0.01)");
            } else eef_step = req.eef_step;
            if (req.jump_threshold < 0)
            {
                ROS_WARN("Jump threshold not allowed (default = 0)");
            } else jump_threshold = req.jump_threshold;
            
            res.fraction = this->set_trajectory(req.wp, eef_step, jump_threshold);
        }
        
        if (res.fraction > 0.2)
        {
            ROS_INFO("Fraction: %f", res.fraction);
            success = true;
        }
    }
    return success;
}

bool ArmControl::execute_target(armcontrolmoveit::ExecuteTargetRequest &req, armcontrolmoveit::ExecuteTargetResponse &res)
{
    this->execute();
    ROS_INFO("Trajectory executed");
    return true;
}

bool ArmControl::demo_precision(armcontrolmoveit::DemoPrecisionRequest &req, armcontrolmoveit::DemoPrecisionResponse &res)
{
    ROS_INFO("ROS Service: DEMO PRECISION");

    // Variable init
    geometry_msgs::Pose target;
    ros::Duration d(3);

    // Move to home position
    this->move_to_point(this->home);
    this->execute();
    switch (req.axis)
    {
        case axis_enum::X:
            target.position.x += req.distance;
            break;
        case axis_enum::Y:
            target.position.y += req.distance;
            break;
        case axis_enum::Z:
            target.position.z += req.distance;
            break;
        case axis_enum::BD:
            target.position.x += req.distance;
            target.position.y -= req.distance;
            target.position.z -= req.distance;
            break;
        case axis_enum::BI:
            target.position.x += req.distance;
            target.position.y += req.distance;
            target.position.z -= req.distance;
            break;
        case axis_enum::AD:
            target.position.x += req.distance;
            target.position.y -= req.distance;
            target.position.z += req.distance;
            break;
        case axis_enum::AI:
            target.position.x += req.distance;
            target.position.y += req.distance;
            target.position.z += req.distance;
    }

    // Demo start
    for (int i = 0; i < req.repetitions; i++)
    {
        this->move_to_point(target);
        this->execute();
        d.sleep();

        this->move_to_point(this->home);
        this->execute();
        d.sleep();
    }
    res.response = true;
    return true;
}

bool ArmControl::home_service(armcontrolmoveit::HomeServiceRequest &req, armcontrolmoveit::HomeServiceResponse &res)
{
    bool success = false;
    this->updateHome();
    if (this->move_to_point(this->home))
    {
        ROS_INFO("Going home position...");
        ROS_INFO("Executing...");
        this->execute();
        ROS_INFO("Goal achieved!");
        success = true;
    } else {
        ROS_ERROR("Error at setting position target...");
    }
    return success;
}

/*
* *************           *************
*               Get & Set
* *************           *************
*/

moveit::planning_interface::MoveGroupInterface::Plan ArmControl::getPlan()
{
    return this->my_plan;
}

const moveit::core::JointModelGroup* ArmControl::getJointModelGroup()
{
    return this->ptr_move_group->getCurrentState()->getJointModelGroup(this->planning_group.c_str());
}