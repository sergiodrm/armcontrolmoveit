#!/usr/bin/env python

import rospy
from armcontrolmoveit.srv import *
import numpy as np
import trajectory_msgs.msg
import dynamixel_workbench_msgs.msg
import actionlib
import kinova_msgs.msg
import sensor_msgs.msg

SUCCESS = 0
JOINT_DIFF_ERROR = 1
PLANNING_ERROR = 2
DRAW_DOOR_ERROR = 3
EXECUTION_ERROR = 4

OPEN_GRIPPER = 0
CLOSE_GRIPPER = 1.5

TorsoInMotion = False
JointValues = [0, 0, 0, 0, 0, 0, 0, 0, 0]

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

def setJointValuesService(req):
    srvName = "/arm/set_joint_values"
    try:
        print("Esperando al servicio " + srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, SetJointValues)
        return srv(req)
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False

def loadParameter(name, defaultValue):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("Parametro " + name+ " no especificado, valor por defecto: " + str(defaultValue))
        return defaultValue


def GoToPoint(point, rpy):
    resChangeTarget = changeTarget(
        ChangeTargetRequest(
            point[0],
            point[1],
            point[2],
            rpy[0],
            rpy[1],
            rpy[2],
            0
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

######################################################################
### Funcion para publicar en el topico encargado de mover el torso ###
######################################################################
def TorsoJointTrajectory(position=0, joint='torso_slider_joint'):
    ## Variables de ROS necesarias
    rate = rospy.Rate(1)

    # Suscripcion para detectar cuando el torso ha dejado de moverse.
    # La llamada a esta funcion asume que cuando siga el hilo de ejecucion
    # el torso ya esta en la posicion deseada, por lo que tiene que comprobar
    # que los motores han dejado de moverse
    sub = rospy.Subscriber("/rb1/torso_controllers/dynamixel_state", 
                            dynamixel_workbench_msgs.msg.DynamixelStateList, 
                            TorsoMotionCallback)
    # Para publicar en el topico correspondiente
    TorsoPub = rospy.Publisher('/rb1/torso_controllers/joint_trajectory',
                                 trajectory_msgs.msg.JointTrajectory, queue_size=10)

    ## Crear el mensaje
    TorsoMsg = trajectory_msgs.msg.JointTrajectory()
    TorsoPoint = trajectory_msgs.msg.JointTrajectoryPoint()
    TorsoPoint.positions.append(position)
    TorsoMsg.header.stamp = rospy.Time.now()
    TorsoMsg.joint_names.append(joint)
    TorsoMsg.points.append(TorsoPoint)
    rate.sleep()

    ## Publicar
    TorsoPub.publish(TorsoMsg)
    rate.sleep()

    ## Comprobar cuando deja de moverse el torso
    while TorsoInMotion:
        rate.sleep()


def TorsoMotionCallback(msg):
    global TorsoInMotion
    for state in msg.dynamixel_state:
        if state.id == 1:
            if -30 < state.present_velocity < 30:
                TorsoInMotion = False
                
            else:
                TorsoInMotion = True

# def JointStateCallback(msg):
#     if msg.name[0] == 'j2s7s200_joint_1':
#         global JointValues
#         for i in range(len(msg.position)):
#             JointValues[i] = msg.position[i] * 180 / np.pi

# def SetJointValuesFunction(positions=[180, 320, 0, 320, 0, 180, 0]):
#     ## Inicializar variables de ros
#     rate = rospy.Rate(1)
#     sub = rospy.Subscriber('/rb1/joint_states', sensor_msgs.msg.JointState, JointStateCallback)
#     rate.sleep()
#     client = actionlib.SimpleActionClient('/rb1/j2s7s200_driver/joints_action/joint_angles', 
#                                             kinova_msgs.msg.ArmJointAnglesAction)
#     client.wait_for_server()

#     ## Comprobar estado de las articulaciones para no dar vueltas de mas
#     for i in range(len(positions)):
#         if not -1 < np.round(JointValues[i]) / 360 < 1:
#             positions[i] += np.floor( np.round(JointValues[i]) / 360 ) * 360
#     print(str(positions))
#     print(str(JointValues))
#     raw_input("lkfvlkdkk")

#     goal = kinova_msgs.msg.ArmJointAnglesActionGoal()
#     goal.goal.angles.joint1 = positions[0]
#     goal.goal.angles.joint2 = positions[1]
#     goal.goal.angles.joint3 = positions[2]
#     goal.goal.angles.joint4 = positions[3]
#     goal.goal.angles.joint5 = positions[4]
#     goal.goal.angles.joint6 = positions[5]
#     goal.goal.angles.joint7 = positions[6]
    

#     client.send_goal_and_wait(goal.goal)
#     print(str(client.get_result()))
    