/*
 * DoorServices.cpp
*/

#include <ros/ros.h>
#include <armcontrolmoveit/Door.h>

// Provisional para inicializar la puerta
Door crearPuerta(float giro, geometry_msgs::Point pos);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_services");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(2);

    ros::NodeHandle nh;

    /* Crear objeto puerta: primero generar puntos de definicion */
    geometry_msgs::Point p;
    float giro;
    ros::param::get("/door/ejepuerta/x", p.x);
    ros::param::get("/door/ejepuerta/y", p.y);
    ros::param::get("/door/ejepuerta/z", p.z);
    ros::param::get("/door/giro", giro);

    Door mipuerta = crearPuerta(giro, p);

    ros::ServiceServer server1 = nh.advertiseService("door/generar_trayectoria", &Door::generarTrayectoria, &mipuerta);
    ROS_INFO("Service server /door/generar_trayectorias ready!");
    ros::ServiceServer server2 = nh.advertiseService("door/girar_sistema", &Door::girarSistema, &mipuerta);
    ROS_INFO("Service server /door/girar_sistema ready!");
    ros::ServiceServer server3 = nh.advertiseService("/door/posicion_apoyo", &Door::posicionApoyo, &mipuerta);
    ROS_INFO("Service server /door/posicion_apoyo ready");

    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    ros::shutdown();
    return 0;
}

Door crearPuerta(float giro, geometry_msgs::Point pos)
{
    geometry_msgs::Point picaporte, apoyo;
    float ancho = 0.77, prof = 0.07, largo = 0.11;
    float alpha = atan(prof/ancho), beta = atan(prof/(ancho-largo));
    picaporte.x = pos.x+sqrt(pow(ancho,2)+pow(prof,2))*cos(giro+alpha);
    picaporte.y = pos.y+sqrt(pow(ancho,2)+pow(prof,2))*sin(giro+alpha);
    picaporte.z = pos.z;
    apoyo.x = pos.x+sqrt(pow(ancho-largo,2)+pow(prof,2))*cos(giro+beta);
    apoyo.y = pos.y+sqrt(pow(ancho-largo,2)+pow(prof,2))*sin(giro+beta);
    apoyo.z = pos.z;

    std::cout << "Puerta: [" << pos.x << ", " << pos.y << ", " << pos.z << "]\n";
    std::cout << "Picaporte: [" << picaporte.x << ", " << picaporte.y << ", " << picaporte.z << "]\n";
    std::cout << "Apoyo: [" << apoyo.x << ", " << apoyo.y << ", " << apoyo.z << "]\n";
    
    return Door(apoyo, picaporte, pos);
}