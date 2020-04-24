/*

*/


#include <armcontrolmoveit/VisualTools.h>
#include <armcontrolmoveit/Door.h>
#include <armcontrolmoveit/ArmControl.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_services");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ArmControl arm;

    ros::NodeHandle nh;

    ros::ServiceServer srv1 = nh.advertiseService("/arm/change_target", &ArmControl::change_target, &arm);
    ROS_INFO("Service /arm/change_target ready");
    ros::ServiceServer srv2 = nh.advertiseService("/arm/execute_target", &ArmControl::execute_target, &arm);
    ROS_INFO("Service /arm/execute_target ready");
    ros::ServiceServer srv3 = nh.advertiseService("/arm/demo_precision", &ArmControl::demo_precision, &arm);
    ROS_INFO("Service /arm/demo_precision ready"); 
    ros::ServiceServer srv4 = nh.advertiseService("/arm/home_service", &ArmControl::home_service, &arm);
    ROS_INFO("Service /arm/home_service ready");
    ros::ServiceServer srv5 = nh.advertiseService("/arm/plan_trajectory", &ArmControl::plan_trajectory, &arm);
    ROS_INFO("Service /arm/plan_trajectory ready");

    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, &ArmControl::publishCartesianStates, &arm);
    ROS_INFO("Suscripcion al topico /joint_states");
    

    while (ros::ok())
    {
        ros::spinOnce();
    }

    ROS_INFO("Shutdown ROS...");
    ros::shutdown();
    return 0;
}


