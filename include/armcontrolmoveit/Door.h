/*
* Door.h
*/


#ifndef DOOR_H_
#define DOOR_H_

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <tf/tf.h>
#include <armcontrolmoveit/VisualTools.h>
#include <armcontrolmoveit/GenerarTrayectoriaPuerta.h>
#include <armcontrolmoveit/GirarSistemaPuerta.h>
#include <armcontrolmoveit/PosicionApoyo.h>

using namespace std;
using namespace Eigen;

class ArmControl;
class VisualTools;

enum sistemas {EJEPUERTA, EJEPICAPORTE1, EJEPICAPORTE2, APOYO};

class Door
{
    public:

    Door(geometry_msgs::Point apoyo, geometry_msgs::Point ejepicaporte, geometry_msgs::Point ejepuerta);
    virtual ~Door();

    void drawInRViz();
    void normalizarAngulo(float &angulo);

    float getWidthDoor();
    float getHeightDoor();
    float getDepthLatch();
    float getWidthLatch();
    VisualTools* getvstool();

    bool allowedRotation(float angulo);

    geometry_msgs::Pose getSystemPose(int index);
    geometry_msgs::Point getSystemPoint(int index);
    float getSystemAngle(int index);


    /* Services */
    bool generarTrayectoria(armcontrolmoveit::GenerarTrayectoriaPuertaRequest &req, armcontrolmoveit::GenerarTrayectoriaPuertaResponse &res);
    bool girarSistema(armcontrolmoveit::GirarSistemaPuertaRequest &req, armcontrolmoveit::GirarSistemaPuertaResponse &res);
    bool posicionApoyo(armcontrolmoveit::PosicionApoyoRequest &req, armcontrolmoveit::PosicionApoyoResponse &res);


    /* Operadores */
    Affine3d operator [](int index);

    /* MISC */
    float distancia_euclidea(geometry_msgs::Point p, geometry_msgs::Point q);
    void transform2pose(Affine3d t, geometry_msgs::Pose &pose);

    private:

    std::vector<Affine3d> puerta;
    std::vector<float> angulo_actual;
    VisualTools *vstool;

    /* Medidas de la puerta [m] */
    float width_door = 0.77;
    float height_door = 1.04;
    float depth_latch = 0.07;
    float width_latch = 0.1;
    float max_angle_latch = M_PI/6; // radians
};

#endif