#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix
from armcontrolmoveit.srv import *


def clienteSrvHome():
    try:
        print("Esperando al servicio: /arm/home_service")
        rospy.wait_for_service('/arm/home_service')
        srv = rospy.ServiceProxy('/arm/home_service', HomeService)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio /arm/home_service fallida: " + str(e))
    return False

def pedirApoyo():
    try:
        print("Esperando al servicio: /door/posicion_apoyo")
        rospy.wait_for_service('/door/posicion_apoyo')
        srv = rospy.ServiceProxy('/door/posicion_apoyo', PosicionApoyo)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio /door/posicion_apoyo fallida: " + str(e))
    return False

def changeTarget(punto):
    try:
        print("Esperando al servicio: /arm/change_target")
        rospy.wait_for_service('/arm/change_target')
        srv = rospy.ServiceProxy('/arm/change_target', ChangeTarget)
        return srv(punto)
    except rospy.ServiceException, e:
        print("Llamada a servicio /arm/change_target fallida: " + str(e))
    return False

def executeTarget():
    try:
        print("Esperando al servicio: /arm/execute_target")
        rospy.wait_for_service('/arm/execute_target')
        srv = rospy.ServiceProxy('/arm/execute_target', ExecuteTarget)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio /arm/execute_target fallida: " + str(e))
    return False

def generarTrayectoria(type_trajectory):
    try:
        print("Esperando al servicio: /door/generar_trayectoria")
        rospy.wait_for_service('/door/generar_trayectoria')
        srv = rospy.ServiceProxy('/door/generar_trayectoria', GenerarTrayectoriaPuerta)
        return srv(type_trajectory)
    except rospy.ServiceException, e:
        print("Llamada a servicio /door/generar_trayectoria fallida: " + str(e))
    return False

def girarSistema(trans):
    try:
        print("Esperando al servicio: /door/girar_sistema")
        rospy.wait_for_service('/door/girar_sistema')
        srv = rospy.ServiceProxy('/door/girar_sistema', GirarSistemaPuerta)
        return srv(trans)
    except rospy.ServiceException, e:
        print("Llamada a servicio /door/girar_sistema fallida: " + str(e))
    return False

def planificarTrayectoria(trajectory):
    try:
        print("Esperando al servicio: /arm/plan_trajectory")
        rospy.wait_for_service('/arm/plan_trajectory')
        srv = rospy.ServiceProxy('/arm/plan_trajectory', PlanTrajectory)
        return srv(trajectory)
    except rospy.ServiceException, e:
        print("Llamada a servicio /arm/plan_trajectory fallida: " + str(e))
    return False

def drawDoorRViz():
    try:
        print("Esperando al servicio: /door/drawDoorRViz")
        rospy.wait_for_service('/door/drawDoorRViz')
        srv = rospy.ServiceProxy('/door/drawDoorRViz', DrawDoorRViz)
        return srv()
    except rospy.ServiceException, e:
        print("Llamada a servicio /door/drawDoorRViz fallida: " + str(e))
    return False


def abrirPuerta():
    # Primero llevar el brazo a la posicion home
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    if not clienteSrvHome():
        return False
    if not drawDoorRViz():
        return False

    # Pedir posicion de apoyo y prepararse para coger el picaporte
    apoyo = pedirApoyo()
    print(type(apoyo))
    if type(apoyo) is bool and not apoyo:
        return False
    q = (apoyo.pose.orientation.x,
         apoyo.pose.orientation.y,
         apoyo.pose.orientation.z,
         apoyo.pose.orientation.w)
    rpy = euler_from_quaternion(q)
    pos_prep = quaternion_matrix(q)
    pos_prep[0:3, 3] = [apoyo.pose.position.x,
                        apoyo.pose.position.y,
                        apoyo.pose.position.z]
    pos_prep = np.dot(pos_prep, np.array([[1, 0, 0, 0],
                                          [0, 1, 0, 0],
                                          [0, 0, 1, -0.1],
                                          [0, 0, 0, 1]]))

    punto = ChangeTargetRequest(pos_prep[0, 3],
                                pos_prep[1, 3],
                                pos_prep[2, 3],
                                rpy[0], rpy[1], rpy[2], 0)
    print(str(punto))
    if not changeTarget(punto):
        return False
    if not drawDoorRViz():
        return False
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    if not executeTarget():
        return False

    # Desplazarse a la posicion del picaporte y cerrar la pinza
    punto = ChangeTargetRequest(apoyo.pose.position.x,
                                apoyo.pose.position.y,
                                apoyo.pose.position.z,
                                rpy[0], rpy[1], rpy[2], 0)
    if not changeTarget(punto):
        return False
    if not drawDoorRViz():
        return False
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    if not executeTarget():
        return False

    # Cerrar pinza

    # Pedir trayectoria de giro del picaporte, planificarla y ejecutarla
    if rospy.has_param('/door/apertura/picaporte/giro'):
        giro_picaporte = rospy.get_param('/door/apertura/picaporte/giro')
        if giro_picaporte > 0:
            print("Valor especificado de /door/apertura/picaporte/giro no valido!")
            return False
    else:
        print("Parametro /door/apertura/picaporte/giro no especificado, valor por defecto: -pi/6")
        giro_picaporte = -np.pi / 6
    if rospy.has_param('/door/apertura/picaporte/numero_puntos'):
        numero_puntos_picaporte = rospy.get_param('/door/apertura/picaporte/numero_puntos')
    else:
        print("Parametro /door/apertura/picaporte/numero_puntos no especificado, valor por defecto: 10")
        numero_puntos_picaporte = 10

    if rospy.has_param('/door/apertura/puerta/giro'):
        giro_puerta = rospy.get_param('/door/apertura/puerta/giro')
    else:
        print("Parametro /door/apertura/puerta/giro no especificado, valor por defecto: -pi/6")
        giro_puerta = -np.pi / 6
    if rospy.has_param('/door/apertura/puerta/numero_puntos'):
        numero_puntos_puerta = rospy.get_param('/door/apertura/puerta/numero_puntos')
    else:
        print("Parametro /door/apertura/puerta/numero_puntos no especificado, valor por defecto: 10")
        numero_puntos_puerta = 10
    req_generarTrayectoria = [GenerarTrayectoriaPuertaRequest(giro_picaporte, numero_puntos_picaporte, 1),
                              GenerarTrayectoriaPuertaRequest(giro_puerta, numero_puntos_puerta, 0),
                              GenerarTrayectoriaPuertaRequest(-giro_picaporte, numero_puntos_picaporte, 1)]
    req_girarSistema = [GirarSistemaPuertaRequest(giro_picaporte, 1),
                        GirarSistemaPuertaRequest(giro_puerta, 0),
                        GirarSistemaPuertaRequest(-giro_picaporte, 1)]
    for i in range(len(req_generarTrayectoria)):
        tray = generarTrayectoria(req_generarTrayectoria[i])
        if type(tray) is bool and not tray:
            return False
        req_trayectoria = PlanTrajectoryRequest(wp=tray.wp, type=1, planning_time=5, eef_step=0.1, jump_threshold=0)
        fraction = planificarTrayectoria(req_trayectoria)
        if type(fraction) is bool and not fraction:
            return False
        if not drawDoorRViz():
            return False
        raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
        if not executeTarget():
            return False
        if not girarSistema(req_girarSistema[i]):
            return False

    # Abrir pinza
    return True


if __name__ == '__main__':
    rospy.init_node("opening_door")
    rospy.wait_for_service("/arm/home_service")
    if abrirPuerta():
        print("Goal achieved!")
    else:
        print("Something was wrong :(")
    print("*** Fin del nodo /opening_door ***")
