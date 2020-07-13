#!/usr/bin/env python

import rospy
from armcontrolmoveit.srv import *
import numpy as np

SUCCESS = 0
JOINT_DIFF_ERROR = 1
PLANNING_ERROR = 2
DRAW_DOOR_ERROR = 3
EXECUTION_ERROR = 4

OPEN_GRIPPER = 0
CLOSE_GRIPPER = 1.5


def clienteSrvHome():
    srvName = "/arm/home_service"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, HomeService)
        res = srv()
        if not res.errorCode == 0:
            print("Error en %s. Codigo de error %d" % (srvName, res.errorCode))
        print(str(res))
        return res
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        res = HomeServiceResponse
        res.success = False
        return res


def getSupport():
    srvName = "/door/support_position"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, PosicionApoyo)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        return False


def changeTarget(punto):
    srvName = "/arm/change_target"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, ChangeTarget)
        res = srv(punto)
        if not res.errorCode == 0:
            print("Error en %s. Codigo de error %d" % (srvName, res.errorCode))
        return res
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        res = ChangeTargetResponse
        res.success = False
        return res


def executeTarget():
    srvName = "/arm/execute_target"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, ExecuteTarget)
        res = srv()
        if not res.errorCode == 0:
            print("Error en %s. Codigo de error %d" % (srvName, res.errorCode))
        return res
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        res = ExecuteTargetResponse
        res.success = False
        return res


def generateTrajectory(type_trajectory):
    srvName = "/door/generate_system"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, GenerateDoorTrajectory)
        return srv(type_trajectory)
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False


def rotateSystem(trans):
    srvName = "/door/rotate_system"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, RotateSystem)
        return srv(trans)
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False


def planTrajectory(trajectory):
    srvName = "/arm/plan_trajectory"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, PlanTrajectory)
        res = srv(trajectory)
        if not res.errorCode == 0:
            print("Error en %s. Codigo de error %d" % (srvName, res.errorCode))
        return res
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        res = PlanTrajectoryResponse
        res.success = False
        return res


def drawDoorRViz():
    srvName = "/door/drawDoorRViz"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, DrawDoorRViz)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False

def loadParameter(name, defaultValue):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("Parametro " + name+ " no especificado, valor por defecto: " + str(defaultValue))
        return defaultValue


def GoToPoint(point, rpy, planning_time=10):
    resChangeTarget = changeTarget(
        ChangeTargetRequest(
            point[0],
            point[1],
            point[2],
            rpy[0],
            rpy[1],
            rpy[2],
            0,
            planning_time
        )
    )
    if not resChangeTarget.success and not resChangeTarget.errorCode == JOINT_DIFF_ERROR:
        print("Error resChangeTarger.errorCode" + str(resChangeTarget.errorCode))
        return resChangeTarget.errorCode
    # Dibujar en RViz
    if not drawDoorRViz():
        print("Error drawDoorRViz")
        pass
        # return DRAW_DOOR_ERROR
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    resExecuteTarget = executeTarget()
    if not resExecuteTarget.errorCode == SUCCESS:
        if resExecuteTarget.errorCode == JOINT_DIFF_ERROR:
            print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
        else:
            return EXECUTION_ERROR
    return SUCCESS
