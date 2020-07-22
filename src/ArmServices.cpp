/*

*/


#include <armcontrolmoveit/VisualTools.h>
#include <armcontrolmoveit/Door.h>
#include <armcontrolmoveit/ArmControl.h>
#include <armcontrolmoveit/UsefulFunctions.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_services");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* Comprobar si se ha especificado algun namespace
    en la llamada al programa */
    ArmControl* arm;
    std::string ns;
    if (argc > 1)
    {
        ROS_INFO("Se ha especificado el namespace: %s/", argv[1]);
        arm = new ArmControl(argv[1]);
        ns = argv[1];
    } else {
        ROS_INFO("No se ha especificado ningun namespace");
        arm = new ArmControl();
    }

    ros::NodeHandle nh;

    ros::ServiceServer srv1 = nh.advertiseService("/arm/change_target", &ArmControl::changeTarget, arm);
    ROS_INFO("Service /arm/change_target ready");
    ros::ServiceServer srv2 = nh.advertiseService("/arm/execute_target", &ArmControl::executeTarget, arm);
    ROS_INFO("Service /arm/execute_target ready");
    ros::ServiceServer srv3 = nh.advertiseService("/arm/demo_precision", &ArmControl::demoPrecision, arm);
    ROS_INFO("Service /arm/demo_precision ready"); 
    ros::ServiceServer srv4 = nh.advertiseService("/arm/home_service", &ArmControl::homeService, arm);
    ROS_INFO("Service /arm/home_service ready");
    ros::ServiceServer srv5 = nh.advertiseService("/arm/plan_trajectory", &ArmControl::planTrajectory, arm);
    ROS_INFO("Service /arm/plan_trajectory ready");
    ros::ServiceServer srv6 = nh.advertiseService("/arm/set_joint_values", &ArmControl::setJointValues, arm);
    ROS_INFO("Service /arm/set_joint_values ready");

    std::string topic = "/joint_states";
    if (!ns.empty())
        topic = myStrCat(ns, topic);
    ros::Subscriber sub1 = nh.subscribe(topic, 1000, &ArmControl::publishCartesianStates, arm);
    ros::Subscriber sub2 = nh.subscribe(topic, 1000, &ArmControl::jointStatesCallback, arm);
    ROS_INFO("Suscripcion al topico %s", topic.c_str());
    
    std::cout << "\n\033[32;4mArmControl services ready!\033[0m\n\n";
    while (ros::ok())
    {
        ros::spinOnce();
    }

    ROS_INFO("Shutdown ROS...");
    ros::shutdown();
    return 0;
}


