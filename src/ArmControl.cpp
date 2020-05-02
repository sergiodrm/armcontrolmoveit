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

    this->pub_cartesian_plan = nh.advertise<armcontrolmoveit::PoseArrayStamped>("trajectory/cartesian_plan", 1000);
    ROS_INFO("Publicando datos de planificacion en coordenadas cartesianas en /trajectory/cartesian_plan");
    this->pub_cartesian_plan_NotArray = nh.advertise<geometry_msgs::PoseStamped>("trajectory/cartesian_plan_NotArray", 1000);
    ROS_INFO("Publicando datos de planificacion en coordenadas cartesianas punto a punto en /trajectory/cartesian_plan_NotArray");
    this->pub_joint_plan = nh.advertise<trajectory_msgs::JointTrajectory>("trajectory/joint_plan", 1000);
    ROS_INFO("Publicando datos de planificacion en coordenadas articulares en /trajectory/joint_plan");
    this->pub_cartesian_states = nh.advertise<geometry_msgs::PoseStamped>("arm/cartesian_states", 1000);
    ROS_INFO("Publicando datos de /joint_states en coordenadas cartesianas en /arm/cartesian_states");

    robot_model_loader::RobotModelLoader robot_loader("robot_description");
    this->kinematic_plan.first = robot_loader.getModel();
    this->kinematic_plan.second = new robot_state::RobotState(this->kinematic_plan.first);
    this->kinematic_joint.first = robot_loader.getModel();
    this->kinematic_joint.second = new robot_state::RobotState(this->kinematic_joint.first);
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
    this->publishJointPlanTrajectory();
    this->publishCartesianPlanTrajectory();
        
    this->ptr_move_group->execute(this->my_plan);
}

void ArmControl::publishCartesianStates(const sensor_msgs::JointState &joint_msg)
{
    geometry_msgs::PoseStamped msg;
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_joint.first->getJointModelGroup(this->planning_group.c_str());
    const std::vector<std::string> joint_names = joint_model_group->getJointModelNames();
    int j = 0;
    for (int i = 0; i < joint_msg.name.size(); i++)
    {
        while (j < joint_names.size() && joint_msg.name[i].compare(joint_names[j]) != 0)
        {
            j++;
        }
        if (j < joint_names.size())
        {
            
            // joint_values[j++] = joint_msg.position[i];
            this->kinematic_joint.second->setJointPositions(joint_names[j++], &joint_msg.position[i]);
        }
    }
    
    const Eigen::Affine3d end_effector = this->kinematic_joint.second->getFrameTransform("j2s7s300_end_effector");
    
    Eigen::Quaterniond q(end_effector.rotation());
    q.normalize();
    msg.header.stamp = joint_msg.header.stamp;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.pose.position.x = end_effector(0, 3);
    msg.pose.position.y = end_effector(1, 3);
    msg.pose.position.z = end_effector(2, 3);
    this->pub_cartesian_states.publish(msg);
}

void ArmControl::publishCartesianPlanTrajectory()
{
    armcontrolmoveit::PoseArrayStamped msg;
    msg.header.stamp = ros::Time::now();
    
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_plan.first->getJointModelGroup(this->planning_group.c_str());
    const std::vector<std::string> joint_names = joint_model_group->getJointModelNames();

    for (int i = 0; i < this->my_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        geometry_msgs::PoseStamped data;

        for (int j = 0; j < this->my_plan.trajectory_.joint_trajectory.joint_names.size(); j++)
        {
            this->kinematic_plan.second->setJointPositions(this->my_plan.trajectory_.joint_trajectory.joint_names[j], &this->my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
        }
        const Eigen::Affine3d end_effector = this->kinematic_plan.second->getFrameTransform("j2s7s300_end_effector");
        Eigen::Quaterniond q(end_effector.rotation());
        q.normalize();
        data.header.stamp.sec = msg.header.stamp.sec + this->my_plan.trajectory_.joint_trajectory.points.at(i).time_from_start.sec;
        data.header.stamp.nsec = msg.header.stamp.nsec + this->my_plan.trajectory_.joint_trajectory.points.at(i).time_from_start.nsec;
        data.pose.orientation.x = q.x();
        data.pose.orientation.y = q.y();
        data.pose.orientation.z = q.z();
        data.pose.orientation.w = q.w();
        data.pose.position.x = end_effector(0, 3);
        data.pose.position.y = end_effector(1, 3);
        data.pose.position.z = end_effector(2, 3);
        msg.pose.push_back(data);
        this->pub_cartesian_plan_NotArray.publish(data);
    }
    this->pub_cartesian_plan.publish(msg);
}

void ArmControl::publishJointPlanTrajectory()
{
    this->my_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
    this->pub_joint_plan.publish(this->my_plan.trajectory_.joint_trajectory);
}


/*
* *********                     ************
*           Services Callbacks
* *********                     ************
*/

bool ArmControl::changeTarget(armcontrolmoveit::ChangeTargetRequest &req, armcontrolmoveit::ChangeTargetResponse &res)
{
    tf::Quaternion q;
    
    if (req.angles_mode)
    {
        q.setRPY(req.roll*M_PI/180, req.pitch*M_PI/180, req.yaw*M_PI/180);
    } else q.setRPY(req.roll, req.pitch, req.yaw);
    
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

bool ArmControl::planTrajectory(armcontrolmoveit::PlanTrajectoryRequest &req, armcontrolmoveit::PlanTrajectoryResponse &res)
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

bool ArmControl::executeTarget(armcontrolmoveit::ExecuteTargetRequest &req, armcontrolmoveit::ExecuteTargetResponse &res)
{
    this->execute();
    ROS_INFO("Trajectory executed");
    return true;
}

bool ArmControl::demoPrecision(armcontrolmoveit::DemoPrecisionRequest &req, armcontrolmoveit::DemoPrecisionResponse &res)
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

bool ArmControl::homeService(armcontrolmoveit::HomeServiceRequest &req, armcontrolmoveit::HomeServiceResponse &res)
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
