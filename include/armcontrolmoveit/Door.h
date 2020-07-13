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
#include <armcontrolmoveit/GenerateDoorTrajectory.h>
#include <armcontrolmoveit/RotateSystem.h>
#include <armcontrolmoveit/SupportPosition.h>
#include <armcontrolmoveit/DrawDoorRViz.h>
#include <armcontrolmoveit/UsefulFunctions.h>

using namespace std;
using namespace Eigen;

class ArmControl;
class VisualTools;

enum sistemas {EJEPUERTA, EJEPICAPORTE1, EJEPICAPORTE2, APOYO};

class Door
{
    public:

    Door(geometry_msgs::Point apoyo, geometry_msgs::Point ejepicaporte, geometry_msgs::Point ejepuerta, std::string referenceFrame="/rb1_base_footprint");
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
    bool generateSystem(armcontrolmoveit::GenerateDoorTrajectoryRequest &req, armcontrolmoveit::GenerateDoorTrajectoryResponse &res);
    bool rotateSystem(armcontrolmoveit::RotateSystemRequest &req, armcontrolmoveit::RotateSystemResponse &res);
    bool supportPosition(armcontrolmoveit::SupportPositionRequest &req, armcontrolmoveit::SupportPositionResponse &res);
    bool drawDoorRViz(armcontrolmoveit::DrawDoorRVizRequest &req, armcontrolmoveit::DrawDoorRVizResponse &res);


    /* Operadores */
    Affine3d operator [](int index);

    private:

    std::vector<Affine3d> doorSystems;
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