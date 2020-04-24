/*

*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <armcontrolmoveit/PlanTrajectory.h>
#include <armcontrolmoveit/ChangeTarget.h>
#include <armcontrolmoveit/ExecuteTarget.h>
#include <armcontrolmoveit/HomeService.h>
#include <armcontrolmoveit/GenerarTrayectoriaPuerta.h>
#include <armcontrolmoveit/GirarSistemaPuerta.h>
#include <armcontrolmoveit/PosicionApoyo.h>
#include <armcontrolmoveit/VisualTools.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pruebas_trayectorias");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(10);
    ros::NodeHandle nh;
    VisualTools vstool;

    ros::ServiceClient srv_generarTrayectoria = nh.serviceClient<armcontrolmoveit::GenerarTrayectoriaPuerta>("/door/generar_trayectoria");
    ROS_INFO("Client /door/generar_trayectoria ready!");
    ros::ServiceClient srv_girarSistema = nh.serviceClient<armcontrolmoveit::GirarSistemaPuerta>("/door/girar_sistema");
    ROS_INFO("Client /door/girar_sistema ready!");
    ros::ServiceClient srv_posicionApoyo = nh.serviceClient<armcontrolmoveit::PosicionApoyo>("/door/posicion_apoyo");
    ROS_INFO("Client /door/posicion_apoyo ready!");

    armcontrolmoveit::GenerarTrayectoriaPuerta generarTrayectoria;
    armcontrolmoveit::GirarSistemaPuerta girarSistema;
    armcontrolmoveit::PosicionApoyo posicionApoyo;

    ros::ServiceClient srv_changeTarget = nh.serviceClient<armcontrolmoveit::ChangeTarget>("/arm/change_target");
    ROS_INFO("Client /arm/change_target ready!");
    ros::ServiceClient srv_executeTarget = nh.serviceClient<armcontrolmoveit::ExecuteTarget>("/arm/execute_target");
    ROS_INFO("Client /arm/execute_target ready!");
    ros::ServiceClient srv_planTrajectory = nh.serviceClient<armcontrolmoveit::PlanTrajectory>("/arm/plan_trajectory");
    ROS_INFO("Client /arm/plan_trajectory ready!");
    ros::ServiceClient srv_homeService = nh.serviceClient<armcontrolmoveit::HomeService>("/arm/home_service");
    ROS_INFO("Client /arm/home_service ready!");

    armcontrolmoveit::ChangeTarget changeTarget;
    armcontrolmoveit::ExecuteTarget executeTarget;
    armcontrolmoveit::PlanTrajectory planTrajectory;
    armcontrolmoveit::HomeService homeService;

    /* Primero a casa */
    vstool.prompt("Pulsar next para ir a la posicion home...");
    if (srv_homeService.call(homeService)) // HOME SERVICE
    {
        ROS_INFO("Home service done!");
        if (srv_posicionApoyo.call(posicionApoyo)) // POSICION APOYO
        {
            ROS_INFO("Planificando trayectoria de posicionamiento...");
            double roll, pitch, yaw;
            tf::Quaternion q;
            
            q.setW(posicionApoyo.response.pose.orientation.w); q.setX(posicionApoyo.response.pose.orientation.x);
            q.setY(posicionApoyo.response.pose.orientation.y); q.setZ(posicionApoyo.response.pose.orientation.z);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            changeTarget.request.x = posicionApoyo.response.pose.position.x;
            changeTarget.request.y = posicionApoyo.response.pose.position.y;
            changeTarget.request.z = posicionApoyo.response.pose.position.z;
            changeTarget.request.roll = roll*180/M_PI;
            changeTarget.request.pitch = pitch*180/M_PI;
            changeTarget.request.yaw = yaw*180/M_PI;
            changeTarget.request.angles_mode = 1;
            

            if (srv_changeTarget.call(changeTarget))
            {
                vstool.prompt("Pulsar next para ejecutar posicionamiento del brazo...");
                if (srv_executeTarget.call(executeTarget))
                {
                    ROS_INFO("Planificando giro de picaporte...");
                    generarTrayectoria.request.angulo = -M_PI/6;
                    generarTrayectoria.request.np = 10;
                    generarTrayectoria.request.sistema = sistemas::EJEPICAPORTE2;
                    if (srv_generarTrayectoria.call(generarTrayectoria))
                    {
                        planTrajectory.request.wp = generarTrayectoria.response.wp;
                        planTrajectory.request.type = type::cartesian;
                        
                        if (srv_planTrajectory.call(planTrajectory))
                        {
                            ROS_INFO("Fraction: %f", planTrajectory.response.fraction);
                            vstool.prompt("Pulsar next para ejecutar movimiento...");
                            if (srv_executeTarget.call(executeTarget))
                            {
                                girarSistema.request.angulo = -M_PI/6;
                                girarSistema.request.sistema = sistemas::EJEPICAPORTE2;
                                if (srv_girarSistema.call(girarSistema))
                                {
                                    ROS_INFO("Sistema girado correctamente.");
                                } else ROS_ERROR("ERROR: GIRAR SISTEMA 1");
                                generarTrayectoria.request.angulo = -M_PI/6;
                                generarTrayectoria.request.np = 10;
                                generarTrayectoria.request.sistema = sistemas::EJEPUERTA;
                                if (srv_generarTrayectoria.call(generarTrayectoria))
                                {
                                    planTrajectory.request.wp = generarTrayectoria.response.wp;
                                    planTrajectory.request.type = type::cartesian;
                                    if (srv_planTrajectory.call(planTrajectory))
                                    {
                                        ROS_INFO("Fraction: %f", planTrajectory.response.fraction);
                                        vstool.prompt("Pulsar next para ejecutar movimiento...");
                                        if (srv_executeTarget.call(executeTarget))
                                        {
                                            girarSistema.request.angulo = -M_PI/6;
                                            girarSistema.request.sistema = sistemas::EJEPUERTA;
                                            if (srv_girarSistema.call(girarSistema))
                                            {
                                                ROS_INFO("Sistema girado correctamente.");
                                            } else ROS_ERROR("ERROR: GIRAR SISTEMA 2");
                                            generarTrayectoria.request.angulo = M_PI/6;
                                            generarTrayectoria.request.np = 10;
                                            generarTrayectoria.request.sistema = sistemas::EJEPICAPORTE2;
                                            if (srv_generarTrayectoria.call(generarTrayectoria))
                                            {
                                                planTrajectory.request.wp = generarTrayectoria.response.wp;
                                                planTrajectory.request.type = type::cartesian;
                                                if (srv_planTrajectory.call(planTrajectory))
                                                {
                                                    ROS_INFO("Fraction: %f", planTrajectory.response.fraction);
                                                    vstool.prompt("Pulsar next para ejecutar movimiento...");
                                                    if (srv_executeTarget.call(executeTarget))
                                                    {
                                                        girarSistema.request.angulo = M_PI/6;
                                                        girarSistema.request.sistema = sistemas::EJEPICAPORTE2;
                                                        if (srv_girarSistema.call(girarSistema))
                                                        {
                                                            ROS_INFO("Sistema girado correctamente.");
                                                        } else ROS_ERROR("ERROR: GIRAR SISTEMA 3");
                                                    } else ROS_ERROR("ERROR: EJECUCION 3");
                                                }
                                            } else ROS_ERROR("ERROR: GENERACION TRAYECTORIA 3");
                                        } else ROS_ERROR("ERROR: EJECUCION 2");
                                    } else ROS_ERROR("ERROR: PLANIFICACION 2");
                                } else ROS_ERROR("ERROR: GENERACION TRAYECTORIA 2");
                            } else ROS_ERROR("ERROR: EJECUCION 1");
                        } else ROS_ERROR("ERROR: PLANIFICACION 1");
                    } else ROS_ERROR("ERROR: GENERACION TRAYECTORIA 1");
                } else ROS_ERROR("ERROR: EJECUCION 0");
            } else ROS_ERROR("ERROR: POSICIONAMIENTO");
        } else ROS_ERROR("ERROR: POSICION APOYO");
    } else ROS_ERROR("ERROR: HOME SERVICE");
    
    
    /* srv.request.wp.clear();
    int salir = 1, cont = 0;
    geometry_msgs::Pose target;
    tf::Quaternion q;
    do {
        float roll, pitch, yaw;
        std::cout << "Punto " << cont++ << std::endl;
        std::cout << "\tx: ";
        std::cin >> target.position.x;
        std::cout << "\ty: ";
        std::cin >> target.position.y;
        std::cout << "\tz: ";
        std::cin >> target.position.z;

        std::cout << "\troll: ";
        std::cin >> roll;
        std::cout << "\tpitch: ";
        std::cin >> pitch;
        std::cout << "\tyaw: ";
        std::cin >> yaw;

        q.setRPY(roll, pitch, yaw);
        q.normalize();
        target.orientation.x = q.getX();
        target.orientation.y = q.getY();
        target.orientation.z = q.getZ();
        target.orientation.w = q.getW();

        srv.request.wp.push_back(target);

        std::cout << "Otro punto? (1/0): ";
        std::cin >> salir;
    } while (salir);

    
    srv.request.eef_step = 0;
    srv.request.jump_threshold = 0;
    srv.request.type = 1;
    if (client.call(srv))
    {
        ROS_INFO("Service called! (response: fraction = %f)", srv.response.fraction);
    } else {
        else ROS_ERROR("Service called failed :(");
    } */

    ros::shutdown();
    return 0;
}
