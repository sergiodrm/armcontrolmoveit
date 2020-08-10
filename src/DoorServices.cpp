/*
 * DoorServices.cpp
*/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <armcontrolmoveit/Door.h>
#include <armcontrolmoveit/ArmControl.h>
#include <armcontrolmoveit/UsefulFunctions.h>

// Provisional para inicializar la puerta
Door crearPuerta(float orientacion, geometry_msgs::Point pos, std::string refFrame, float ancho=0.77, float prof=0.05, float largo=0.10);

// Lectura de topico para crear el modelo de la puerta cuando termine la navegacion
static bool RunNav = true;
void readRunNav(const std_msgs::Bool& msg)
{
    RunNav = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_services");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(2);

    ros::NodeHandle nh;

    std::string refFrame;
    /* Crear objeto puerta: primero generar puntos de definicion */
    geometry_msgs::Point p;
    float orientacion;
    float ancho , prof, largo;
    if (argc > 1)
    {
        ros::param::get(myStrCat(argv[1], "/door/ejepuerta/x").c_str(), p.x);
        ros::param::get(myStrCat(argv[1], "/door/ejepuerta/y").c_str(), p.y);
        ros::param::get(myStrCat(argv[1], "/door/ejepuerta/z").c_str(), p.z);
        ros::param::get(myStrCat(argv[1], "/door/medidas/largo").c_str(), largo);
        ros::param::get(myStrCat(argv[1], "/door/medidas/ancho").c_str(), ancho);
        ros::param::get(myStrCat(argv[1], "/door/medidas/profundo").c_str(), prof);
        ros::param::get(myStrCat(argv[1], "/door/ejepuerta/orientacion").c_str(), orientacion);
        ros::param::get(myStrCat(argv[1], "/rviz/referenceFrame").c_str(), refFrame);
    } else {
        ros::param::get("/door/ejepuerta/x", p.x);
        ros::param::get("/door/ejepuerta/y", p.y);
        ros::param::get("/door/ejepuerta/z", p.z);
        ros::param::get("/door/ejepuerta/orientacion", orientacion);
        ros::param::get("/rviz/referenceFrame", refFrame);
    }

    /* Con todos los parametros cargados, hay que esperar a la finalizacion de la navegacion */
    std::string param_nav = "/running_nav";
    if (ros::param::has(myStrCat(argv[1], param_nav).c_str()))
    {
        ROS_INFO("Esperando que termine la navegacion...");
        bool run;
        do {
            ros::param::get(myStrCat(argv[1], param_nav).c_str(), run);
        } while (run);
    } else {
        ROS_ERROR("Parametro %s no encontrado.", myStrCat(argv[1], param_nav).c_str());
    }

    ROS_INFO("Parametro %s indica que se puede generar el modelo de la puerta.", 
        myStrCat(argv[1], param_nav).c_str());
    ROS_INFO("Generando modelo...");

    /* Generar modelo de la puerta */
    Door mipuerta = crearPuerta(orientacion, p, refFrame, ancho, prof, largo);

    /* Inicializar los servicios de la clase Door */
    ros::ServiceServer server1 = nh.advertiseService("/door/generate_trajectory", &Door::generateSystem, &mipuerta);
    ROS_INFO("Service /door/generate_trajectory ready");
    ros::ServiceServer server2 = nh.advertiseService("/door/rotate_system", &Door::rotateSystem, &mipuerta);
    ROS_INFO("Service /door/rotate_system ready");
    ros::ServiceServer server3 = nh.advertiseService("/door/support_position", &Door::supportPosition, &mipuerta);
    ROS_INFO("Service /door/support_position ready");
    ros::ServiceServer server4 = nh.advertiseService("/door/drawDoorRViz", &Door::drawDoorRViz, &mipuerta);
    ROS_INFO("Service /door/drawDoorRViz ready");

    std::cout << "\n\033[32;4mDoor services ready!\033[0m\n\n";

    while (ros::ok())
        ros::spinOnce();
    
    
    ros::shutdown();
    return 0;
}

Door crearPuerta(float orientacion, geometry_msgs::Point pos, std::string refFrame, float ancho, float prof, float largo)
{
    geometry_msgs::Point picaporte, apoyo;
    
    float alpha = atan2(prof, ancho), beta = atan2(prof, ancho - largo);
    picaporte.x = pos.x - sqrt(pow(ancho, 2) + pow(prof, 2)) * sin(orientacion + alpha);
    picaporte.y = pos.y - sqrt(pow(ancho, 2) + pow(prof, 2)) * cos(orientacion + alpha);
    picaporte.z = pos.z;
    apoyo.x = pos.x - sqrt(pow(ancho - largo, 2) + pow(prof, 2)) * sin(orientacion + beta);
    apoyo.y = pos.y - sqrt(pow(ancho - largo, 2) + pow(prof, 2)) * cos(orientacion + beta);
    apoyo.z = pos.z;

    std::cout << "Puerta: [" << pos.x << ", " << pos.y << ", " << pos.z << "]\n";
    std::cout << "Picaporte: [" << picaporte.x << ", " << picaporte.y << ", " << picaporte.z << "]\n";
    std::cout << "Apoyo: [" << apoyo.x << ", " << apoyo.y << ", " << apoyo.z << "]\n";
    
    return Door(apoyo, picaporte, pos, refFrame);
}