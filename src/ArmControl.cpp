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
    ROS_INFO("Loading ArmControl constructor...");
    ros::param::get("/move_group_config/planning_group", this->planning_group);
    std::cout << "\n\033[32mPlanning group from ROS param server: \033[0m" << this->planning_group.c_str() << "\n\n";
    this->ptr_move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
    std::cout << "\n\033[32mSuccess! :) \033[0m\n\n";
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
    ROS_INFO("Parametros de configuracion cargados correctamente.");
    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    q.normalize();

    this->home_pose.position.x = x;
    this->home_pose.position.y = y;
    this->home_pose.position.z = z;
    this->home_pose.orientation.x = q.getX();
    this->home_pose.orientation.y = q.getY();
    this->home_pose.orientation.z = q.getZ();
    this->home_pose.orientation.w = q.getW();
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
    
    std::cout << "\n\033[32;4mArmControl ready!\033[0m\n\n";
}

ArmControl::ArmControl(char* ns)
{
    ROS_INFO("Loading ArmControl constructor...");
    this->rosNamespace = ns;
    ros::param::get(myStrCat<const char*, const char*>(this->rosNamespace.c_str(), "/move_group_config/planning_group").c_str(), this->planning_group);
    std::cout << "\n\033[32mPlanning group from ROS param server: \033[0m" << this->planning_group.c_str() << "\n\n";
    this->ptr_move_group = new moveit::planning_interface::MoveGroupInterface(this->planning_group.c_str());
    std::cout << "\n\033[32mSuccess! :) \033[0m\n\n";
    ROS_INFO("Move_group connected with planning_group: %s", this->planning_group.c_str());
    // We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", this->ptr_move_group->getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("End effector link: %s", this->ptr_move_group->getEndEffectorLink().c_str());

    double roll, pitch, yaw;
    tf::Quaternion q;
    
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/x").c_str(), this->home_pose.position.x);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/y").c_str(), this->home_pose.position.y);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/z").c_str(), this->home_pose.position.z);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/roll").c_str(), roll);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/pitch").c_str(), pitch);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/yaw").c_str(), yaw);
    std::string param = myStrCat(this->rosNamespace, "/config_arm/home/joint_");
    for (int i = 0; i < this->home_joints.size(); i++)
    {
        double value;
        ros::param::get(myStrCat(param, std::to_string(i+1)).c_str(), value);
        this->home_joints.push_back(value);
    }
    ROS_INFO("Parametros de configuracion cargados correctamente.");

    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    q.normalize();

    this->home_pose.orientation.x = q.getX();
    this->home_pose.orientation.y = q.getY();
    this->home_pose.orientation.z = q.getZ();
    this->home_pose.orientation.w = q.getW();

    std::cout << "\n\033[34mHome position:\033[0m [" << this->home_pose.position.x << ", " << this->home_pose.position.y << ", " << this->home_pose.position.z << "]";
    std::cout << "[" << this->home_pose.orientation.x << ", " << this->home_pose.orientation.y << ", " << this->home_pose.orientation.z << ", " << this->home_pose.orientation.w << "]\n\n";

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

    std::cout << "\n\033[32;4mArmControl ready!\033[0m\n\n";
}

ArmControl::~ArmControl(){
    delete this->ptr_move_group;
    delete this->ptr_rate;
}

/*
*        Methods to move & update position  
*/

bool ArmControl::move_to_point(const geometry_msgs::Pose &target)
{
    /* Leer el parametro de planning time y configurar tiempo de planificacion */
    float planning_time;
    ros::param::get("/rb1/move_group_config/planning_time", planning_time);
    this->ptr_move_group->setPlanningTime(planning_time);

    /* Cambiar el objetivo de moveit y planificar la trayectoria articular */
    this->ptr_move_group->setPoseTarget(target);
    bool success =  this->ptr_move_group->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    /* Graficar trayectoria en RViz*/
    std::vector<geometry_msgs::Pose> wp;
    wp.push_back(target);
    this->vstool->drawTrajectory(wp, *this);
    
    return success;
}

const float ArmControl::set_trajectory(const std::vector<geometry_msgs::Pose> &wp, float eef_step, float jump_threshold)
{
    /* Leer el parametro de planning time y configurarlo en moveit */
    float planning_time;
    ros::param::get("/rb1/move_group_config/planning_time", planning_time);
    this->ptr_move_group->setPlanningTime(planning_time);

    /* Planificar trayectoria*/
    float fraction = this->ptr_move_group->computeCartesianPath(wp, eef_step, jump_threshold, this->my_plan.trajectory_);

    /* Comprobar resultado de la planificacion */
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
    /* Home pose */
    double x, y, z, roll, pitch, yaw;
    tf::Quaternion q;
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/x").c_str(), x);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/y").c_str(), y);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/z").c_str(), z);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/roll").c_str(), roll);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/pitch").c_str(), pitch);
    ros::param::get(myStrCat(this->rosNamespace, "/config_arm/home/yaw").c_str(), yaw);
    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    q.normalize();

    this->home_pose.position.x = x;
    this->home_pose.position.y = y;
    this->home_pose.position.z = z;
    this->home_pose.orientation.x = q.getX();
    this->home_pose.orientation.y = q.getY();
    this->home_pose.orientation.z = q.getZ();
    this->home_pose.orientation.w = q.getW();

    /* Home Joints */
    std::string param = myStrCat(this->rosNamespace, "/config_arm/home/joint_");
    for (int i = 0; i < this->home_joints.size(); i++)
        ros::param::get(myStrCat(param, std::to_string(i+1)).c_str(), this->home_joints[i]);
    
}

int ArmControl::execute()
{
    int errorCode = errorCodeTrajectory::SUCCESS;
    this->publishJointPlanTrajectory();
    this->publishCartesianPlanTrajectory();
    
    errorCode = this->checkPrintJointStates();
    if (errorCode == errorCodeTrajectory::SUCCESS || errorCode == errorCodeTrajectory::JOINT_DIFF)
    {
        this->ptr_move_group->execute(this->my_plan);
        std::cout << "\n---------- Execution summary ----------\n";
        for (int index = 0; index < this->my_plan.trajectory_.joint_trajectory.points.back().positions.size(); index++)
        {
            double jointError = myabs(normalizeAngle(this->my_plan.trajectory_.joint_trajectory.points.back().positions[index]) - normalizeAngle(this->ptr_move_group->getCurrentJointValues()[index]));
            if (jointError >= MAX_DIFF_JOINTS_VALUES)
            {
                std::cout << "\033[33m";
            } else std::cout << "\033[34m";
            std::cout << " - Error joint " << index << ":\033[0m \t" << normalizeAngle(this->my_plan.trajectory_.joint_trajectory.points.back().positions[index]);
            std::cout << " - " << normalizeAngle(this->ptr_move_group->getCurrentJointValues()[index]) << " -> ";
            std::cout << jointError << std::endl;
        }
        std::cout << "\033[34m - Target:\033[0m \t[" << this->cartesianPlan.pose.back().pose.position.x << ", ";
        std::cout << this->cartesianPlan.pose.back().pose.position.y << ", " << this->cartesianPlan.pose.back().pose.position.z << "]\n";

        std::cout << "\033[34m - Real point:\033[0m \t[" << this->ptr_move_group->getCurrentPose().pose.position.x << ", ";
        std::cout << this->ptr_move_group->getCurrentPose().pose.position.y << ", " << this->ptr_move_group->getCurrentPose().pose.position.z << "]\n";
        
        std::cout << "\033[34m - Error:\033[0m \t[";
        std::cout << myabs(this->cartesianPlan.pose.back().pose.position.x - this->ptr_move_group->getCurrentPose().pose.position.x) << ", ";
        std::cout << myabs(this->cartesianPlan.pose.back().pose.position.y - this->ptr_move_group->getCurrentPose().pose.position.y) << ", ";
        std::cout << myabs(this->cartesianPlan.pose.back().pose.position.z - this->ptr_move_group->getCurrentPose().pose.position.z) << "]\n";

        std::cout << "\033[34m - Total error:\033[0m \t";
        std::cout << distancia_euclidea(this->ptr_move_group->getCurrentPose().pose.position, this->cartesianPlan.pose.back().pose.position) << "\n";

        std::cout << "========================================\n";
    }
    return errorCode;
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
            j++;
            
        if (j < joint_names.size())
        {
            // joint_values[j++] = joint_msg.position[i];
            this->kinematic_joint.second->setJointPositions(joint_names[j++], &joint_msg.position[i]);
        }
    }
    
    const Eigen::Affine3d end_effector = this->kinematic_joint.second->getFrameTransform("j2s7s200_end_effector");
    
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
    this->cartesianPlan.header.stamp = ros::Time::now();
    
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_plan.first->getJointModelGroup(this->planning_group.c_str());
    const std::vector<std::string> joint_names = joint_model_group->getJointModelNames();

    for (int i = 0; i < this->my_plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        geometry_msgs::PoseStamped data;

        for (int j = 0; j < this->my_plan.trajectory_.joint_trajectory.joint_names.size(); j++)
        {
            this->kinematic_plan.second->setJointPositions(this->my_plan.trajectory_.joint_trajectory.joint_names[j], &this->my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
        }
        const Eigen::Affine3d end_effector = this->kinematic_plan.second->getFrameTransform("j2s7s200_end_effector");
        Eigen::Quaterniond q(end_effector.rotation());
        q.normalize();
        data.header.stamp.sec = this->cartesianPlan.header.stamp.sec + this->my_plan.trajectory_.joint_trajectory.points.at(i).time_from_start.sec;
        data.header.stamp.nsec = this->cartesianPlan.header.stamp.nsec + this->my_plan.trajectory_.joint_trajectory.points.at(i).time_from_start.nsec;
        data.pose.orientation.x = q.x();
        data.pose.orientation.y = q.y();
        data.pose.orientation.z = q.z();
        data.pose.orientation.w = q.w();
        data.pose.position.x = end_effector(0, 3);
        data.pose.position.y = end_effector(1, 3);
        data.pose.position.z = end_effector(2, 3);
        this->cartesianPlan.pose.push_back(data);
        this->pub_cartesian_plan_NotArray.publish(data);
    }
    this->pub_cartesian_plan.publish(this->cartesianPlan);
}

void ArmControl::publishJointPlanTrajectory()
{
    this->my_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
    this->pub_joint_plan.publish(this->my_plan.trajectory_.joint_trajectory);
}

int ArmControl::checkPrintJointStates()
{
    int errorCode = errorCodeTrajectory::SUCCESS;
    
    for (int index = 0; index < this->ptr_move_group->getCurrentJointValues().size(); index++)
    {
        if (myabs(normalizeAngle(this->ptr_move_group->getCurrentJointValues()[index]) - normalizeAngle(this->my_plan.trajectory_.joint_trajectory.points[0].positions[index])) >= MAX_DIFF_JOINTS_VALUES)
        {
            std::cout << "\033[33m";
            errorCode = errorCodeTrajectory::JOINT_DIFF;
            std::cout << "Valor articular corregido: " << this->my_plan.trajectory_.joint_trajectory.points[0].positions[index];
            std::cout << " -> " << this->ptr_move_group->getCurrentJointValues()[index] << std::endl;
            this->my_plan.trajectory_.joint_trajectory.points[0].positions[index] = this->ptr_move_group->getCurrentJointValues()[index];
        } else {
            std::cout << "\033[34m";
        }
        std::cout << " - Joint " << index << ": \033[0m" << normalizeAngle(this->ptr_move_group->getCurrentJointValues()[index]);
        std::cout << " / " << normalizeAngle(this->my_plan.trajectory_.joint_trajectory.points[0].positions[index]) << std::endl;
    }
    return errorCode;
}

/*
* *********                     ************
*           Services Callbacks
* *********                     ************
*/

bool ArmControl::changeTarget(armcontrolmoveit::ChangeTargetRequest &req, armcontrolmoveit::ChangeTargetResponse &res)
{
    tf::Quaternion q;
    res.errorCode = errorCodeTrajectory::SUCCESS;
    // if (req.planning_time >= 2)
    // {
    //     ROS_INFO("Setting planning time: %f seconds", req.planning_time);
    //     this->ptr_move_group->setPlanningTime(req.planning_time);
    // } else if (req.planning_time > 0 && req.planning_time < 2)
    // {
    //     ROS_WARN("Planning time too low");
    // }
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
    if (!res.success)
    {
        res.errorCode = errorCodeTrajectory::PLAN_ERROR;
    } else {
        std::cout << "---------- Planning summary ----------\n";
        std::cout << "\033[34m - Planning time: \033[0m" << this->ptr_move_group->getPlanningTime() << std::endl;
        std::cout << "\033[34m - Target: \033[0m[" << t.position.x << ", " << t.position.y << ", " << t.position.z << "] [";
        std::cout << req.roll << ", " << req.pitch << ", " << req.yaw << "]" << std::endl;
        std::cout << "------------- Joint check ------------\n";
        res.errorCode = this->checkPrintJointStates();
        std::cout << "\033[34m - Error code: \033[0m" << res.errorCode << std::endl;
        std::cout << "======================================\n";
    }
    return res.success;
}

bool ArmControl::planTrajectory(armcontrolmoveit::PlanTrajectoryRequest &req, armcontrolmoveit::PlanTrajectoryResponse &res)
{
    /* Crear variable de exito de la planificacion */
    bool success = false;
    float eef_step = 0.01, jump_threshold = 0;
    res.errorCode = errorCodeTrajectory::SUCCESS;
    // if (req.planning_time >= 2)
    // {
    //     ROS_INFO("Setting planning time: %f seconds", req.planning_time);
    //     this->ptr_move_group->setPlanningTime(req.planning_time);
    // } else if (req.planning_time > 0 && req.planning_time < 2)
    // {
    //     ROS_WARN("Planning time too low");
    // }

    /* Si la planificacion es articular, 
    mandar el ultimo punto especificado
    al metodo move_to_point. 
    Si la trayectoria es cartesiana se llama al metodo set_trajectory */
    if (req.type == type::articular)
    {
        success = this->move_to_point(req.wp.at(req.wp.size()-1));
    } else if (req.type == type::cartesian)
    {
        if (req.eef_step > 0 && req.jump_threshold >= 0)
        {
            eef_step = req.eef_step;
            jump_threshold = req.jump_threshold;
            res.fraction = this->set_trajectory(req.wp, req.eef_step, req.jump_threshold);
        } else {
            
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
        /* Si el fraction devuelto es menor del 0.2 se considera planificacion fallida */
        if (res.fraction > 0.2)
        {
            std::cout << "---------- Planning summary ----------\n";
            std::cout << "\033[34m - Planning time: \033[0m" << this->ptr_move_group->getPlanningTime() << std::endl;
            std::cout << "\033[34m - eef step: \033[0m" << eef_step << std::endl;
            std::cout << "\033[34m - jump threshold: \033[0m" << jump_threshold << std::endl;
            std::cout << "\033[34m - Fraction: \033[0m" << res.fraction << std::endl;
            std::cout << "------------- Joint check ------------\n";
            res.errorCode = this->checkPrintJointStates();
            std::cout << "\033[34m - Error code: \033[0m" << res.errorCode << std::endl;
            std::cout << "======================================\n";
            success = true;
        }
    }
    if (!success)
        res.errorCode = errorCodeTrajectory::PLAN_ERROR;
    return success;
}

bool ArmControl::executeTarget(armcontrolmoveit::ExecuteTargetRequest &req, armcontrolmoveit::ExecuteTargetResponse &res)
{
    res.errorCode = this->execute();
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
    this->move_to_point(this->home_pose);
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

        this->move_to_point(this->home_pose);
        this->execute();
        d.sleep();
    }
    res.response = true;
    return true;
}

bool ArmControl::homeService(armcontrolmoveit::HomeServiceRequest &req, armcontrolmoveit::HomeServiceResponse &res)
{
    /* Variable de salida que indica el exito de la operacion */
    bool success = false;

    /* Actualizacion del punto home */
    this->updateHome();
    // ROS_INFO("Home position target: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w:%f]",
    //     this->home_pose.position.x, this->home_pose.position.y, this->home_pose.position.z,
    //     this->home_pose.orientation.x, this->home_pose.orientation.y, 
    //     this->home_pose.orientation.z, this->home_pose.orientation.w);
    
    /* Llamada al metodo move_to_point para mover a la posicion home_pose */
    if (this->move_to_point(this->home_pose))
    {
        ROS_INFO("Going home position...");
        ROS_INFO("Executing...");
        res.errorCode = this->execute();
        ROS_INFO("Goal achieved!");
        success = true;
    } else {
        /* Si falla, devolver en la respuesta el error correspondiente */
        res.errorCode = errorCodeTrajectory::PLAN_ERROR;
        ROS_ERROR("Error at setting position target...");
    }
    return success;
}

bool ArmControl::setJointValues(armcontrolmoveit::SetJointValuesRequest &req, armcontrolmoveit::SetJointValuesResponse &res)
{
    /* Leer el parametro de planning time */
    float planning_time;
    ros::param::get("/rb1/move_group_config/planning_time", planning_time);
    this->ptr_move_group->setPlanningTime(planning_time);

    /* Crear modelo del robot para leer posteriormente la posicion resultante*/
    const robot_state::JointModelGroup* joint_model_group =
        this->ptr_move_group->getCurrentState()->getJointModelGroup(this->planning_group.c_str());
    moveit::core::RobotStatePtr current_state = this->ptr_move_group->getCurrentState();

    /* Crear vector de posiciones y copiar datos del request */
    std::vector<double> joint_group_positions, joint_states;
    current_state->copyJointGroupPositions(joint_model_group, joint_states);
    for (int i = 0; i < req.positions.size(); i++)
    {
        joint_group_positions.push_back(req.positions[i]);
    }
    // std::cout << "joint values: ";
    // for (double value : joint_group_positions)
    //     std::cout << value << " ";
    // std::cout << std::endl;

    /* Realizar planificacion con moveit */
    this->ptr_move_group->setJointValueTarget(joint_group_positions);
    res.success = this->ptr_move_group->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    /* Ejecutar trayectoria */
    this->execute();

    /* Lectura de valores articulares para ver el error de posicion*/
    current_state = this->ptr_move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_states);
    for (double value : joint_states)
    {
        res.states.push_back(value);
    }

    return true;
}

/*
* *************                  *************
*                Topic Callback
* *************                  *************
*/
void ArmControl::jointStatesCallback(const sensor_msgs::JointState msg)
{
    if (msg.name[0].compare("j2s7s200_joint_1") == 0)
    {
        if (this->jointStatesMsgs.size() > MAX_QUEUE_SIZE)
            this->jointStatesMsgs.pop();
        this->jointStatesMsgs.push(msg);
    }
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
