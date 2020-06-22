

#include <armcontrolmoveit/VisualTools.h>

VisualTools::VisualTools()
{
    this->ptr_visual_tools = new moveit_visual_tools::MoveItVisualTools("/world");
    this->ptr_visual_tools->deleteAllMarkers();
    this->ptr_visual_tools->loadRemoteControl();
    this->text_pose = Eigen::Affine3d::Identity();
    /* this->text_pose.translation.z() = 1.5; */

    marker_pub = this->nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);    
}

VisualTools::VisualTools(const std::string &reference_string)
{
    this->ptr_visual_tools = new moveit_visual_tools::MoveItVisualTools(reference_string.c_str());
    this->ptr_visual_tools->deleteAllMarkers();
    this->ptr_visual_tools->loadRemoteControl();
    this->text_pose = Eigen::Affine3d::Identity();
    /* this->text_pose.translation.z() = 1.5; */
}

VisualTools::~VisualTools()
{
    
}

moveit_visual_tools::MoveItVisualTools* VisualTools::getPtrVisualTools()
{
    return this->ptr_visual_tools;
}

void VisualTools::prompt(const std::string &str)
{
    this->ptr_visual_tools->prompt(str);
}

void VisualTools::drawTrajectory(std::vector<geometry_msgs::Pose> wp, ArmControl &arm)
{
    ROS_INFO("Showing trajectory in RViz...");
    this->ptr_visual_tools->deleteAllMarkers();
    this->ptr_visual_tools->loadRemoteControl();
    this->ptr_visual_tools->publishAxisLabeled(wp.at(wp.size()-1), "Pose goal");
    this->ptr_visual_tools->publishTrajectoryLine(arm.getPlan().trajectory_, arm.getJointModelGroup());

    if (!wp.empty())
    {
        this->ptr_visual_tools->publishPath(wp, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
        for (std::size_t i = 0; i < wp.size(); ++i)
        {
            this->ptr_visual_tools->publishAxisLabeled(wp[i], "point " + std::to_string(i), rviz_visual_tools::SMALL);
        }
    }
    this->ptr_visual_tools->trigger();
}

void VisualTools::drawDoor(Door &door)
{

    this->ptr_visual_tools->publishAxisLabeled(door.getSystemPose(sistemas::EJEPUERTA), "EJE_PUERTA", rviz_visual_tools::SMALL);
    this->ptr_visual_tools->publishAxisLabeled(door.getSystemPose(sistemas::EJEPICAPORTE1), "EJE_PICAPORTE1", rviz_visual_tools::SMALL);
    this->ptr_visual_tools->publishAxisLabeled(door.getSystemPose(sistemas::EJEPICAPORTE2), "EJE_PICAPORTE2", rviz_visual_tools::SMALL);
    this->ptr_visual_tools->publishAxisLabeled(door.getSystemPose(sistemas::APOYO), "APOYO", rviz_visual_tools::SMALL);
    this->ptr_visual_tools->trigger();
    
    visualization_msgs::Marker limit, line;
    geometry_msgs::Point p;
    

    /* Configurar header del mensaje */
    limit.header.frame_id = line.header.frame_id = this->ptr_visual_tools->getBaseFrame();
    limit.ns = line.ns = "limit_and_lines";
    limit.header.stamp = line.header.stamp = ros::Time::now();
    limit.action = line.action = visualization_msgs::Marker::DELETE;
    limit.pose.orientation.w = line.pose.orientation.w = 1.0;
    limit.id = 0; line.id = 1;
    marker_pub.publish(limit); marker_pub.publish(line);
    limit.action = line.action = visualization_msgs::Marker::ADD;

    /* Configurar visualizacion */
    limit.type = visualization_msgs::Marker::LINE_STRIP;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    limit.color.a = 0.5; limit.color.r = 1; limit.color.g = 0; limit.color.b = 0;
    line.color.a = 0.7; line.color.r = 0.5; line.color.g = 0; line.color.b = 0.5;

    limit.scale.x = 0.02; limit.scale.y = 0.02; limit.scale.z = 0.02;
    line.scale.x = 0.02; line.scale.y = 0.02; line.scale.z = 0.02;

    /* Representacion en RViz */
    p = door.getSystemPoint(sistemas::EJEPUERTA);
    double z = p.z;
    p.z = 0;
    line.points.push_back(p);
    p.z = z;
    line.points.push_back(p);

    p = door.getSystemPoint(sistemas::EJEPICAPORTE1);
    line.points.push_back(p);
    p = door.getSystemPoint(sistemas::EJEPICAPORTE2);
    line.points.push_back(p);
    p = door.getSystemPoint(sistemas::APOYO);
    line.points.push_back(p);    
    
    float r = distancia_euclidea(door.getSystemPoint(sistemas::EJEPICAPORTE1), door.getSystemPoint(sistemas::EJEPUERTA));
    r += 0.05;
    for (double giro = 0; giro < 2*M_PI; giro += 0.01)
    {
        p = door.getSystemPoint(sistemas::EJEPUERTA);
        p.z = 0;
        p.x += r*cos(giro);
        p.y += r*sin(giro);
        limit.points.push_back(p);
    }
    

    marker_pub.publish(limit);
    marker_pub.publish(line);

    
    
}

void VisualTools::deleteAllMarkers()
{
    this->ptr_visual_tools->deleteAllMarkers();
}