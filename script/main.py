#!/usr/bin/env python

import rospy
import sys
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix
from armcontrolmoveit.srv import *
import control_msgs.msg

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


def pedirApoyo():
    srvName = "/door/posicion_apoyo"
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


def generarTrayectoria(type_trajectory):
    srvName = "/door/generar_trayectoria"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, GenerarTrayectoriaPuerta)
        return srv(type_trajectory)
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False


def girarSistema(trans):
    srvName = "/door/girar_sistema"
    try:
        print("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, GirarSistemaPuerta)
        return srv(trans)
    except rospy.ServiceException, e:
        print("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
    return False


def planificarTrayectoria(trajectory):
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


def abrirPuerta():
    # Comprobar si la ejecucion del script se hace desde un namespace
    ns = ""
    if len(sys.argv) > 1:
        ns = sys.argv[1]
        if not ns[-1] == '/':
            ns = ns + '/'
        if not ns[0] == '/':
            ns = '/' + ns
        print("Ejecutando " + sys.argv[0] + " en el namespace " + ns)
    else:
        print("Ejecutando " + sys.argv[0] + " sin namespace")

    # Planning time
    if rospy.has_param(ns + "move_group_config/planning_time"):
        planning_time = rospy.get_param(ns + "move_group_config/planning_time")
    else:
        planning_time = 7

    # Crear variable para publicar en el topico de la pinza
    GripperPub = rospy.Publisher('/rb1/j2s7s200_gripper/gripper_command/goal',
                                 control_msgs.msg.GripperCommandActionGoal, queue_size=10)
    GripperPubMsg = control_msgs.msg.GripperCommandActionGoal()

    

    ############## PRIMERA FASE: IR A POSICION HOME ##############
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    resHomeSrv = clienteSrvHome()
    if not resHomeSrv.errorCode == 0:
        if resHomeSrv.errorCode == 1:
            print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
        else:
            return False
    if not drawDoorRViz():
        return False

    # Abrir la pinza al principio del codigo
    print("Abriendo pinza...")
    GripperPubMsg.goal.command.position = OPEN_GRIPPER
    GripperPub.publish(GripperPubMsg)

    ############## SEGUNDA FASE: POSICIONARSE EN EL PUNTO DE APOYO ##############
    # Primero pedir posicion
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

    # Realizar la planificacion del punto al que se desea ir
    resChangeTarget = changeTarget(
        ChangeTargetRequest(
            pos_prep[0, 3],
            pos_prep[1, 3],
            pos_prep[2, 3],
            rpy[0], rpy[1], rpy[2], 0, planning_time
        )
    )
    if not resChangeTarget.success:
        return False
    # Dibujar la trayectoria en RViz
    if not drawDoorRViz():
        return False

    # Pedir confirmacion para ejecucion
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    resExecuteTarget = executeTarget()
    if not resExecuteTarget.errorCode == 0:
        if resExecuteTarget.errorCode == 1:
            print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
        else:
            return False

    # ################ COGER EL PICAPORTE #####################
    # Desplazarse a la posicion del picaporte y cerrar la pinza
    resChangeTarget = changeTarget(
        ChangeTargetRequest(
            apoyo.pose.position.x,
            apoyo.pose.position.y,
            apoyo.pose.position.z,
            rpy[0], rpy[1], rpy[2],
            0, planning_time
        )
    )
    if not resChangeTarget.success:
        return False
    # Dibujar en RViz
    if not drawDoorRViz():
        return False
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    resExecuteTarget = executeTarget()
    if not resExecuteTarget.errorCode == 0:
        if resExecuteTarget.errorCode == 1:
            print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
        else:
            return False

    # Cerrar pinza
    print("Cerrando pinza...")
    GripperPubMsg.goal.command.position = CLOSE_GRIPPER
    GripperPub.publish(GripperPubMsg)

    # Pedir trayectoria de giro del picaporte, planificarla y ejecutarla
    if rospy.has_param(ns + '/door/apertura/picaporte/giro'):
        giro_picaporte = rospy.get_param(ns + '/door/apertura/picaporte/giro')
        if giro_picaporte > 0:
            print("Valor especificado de giro del picaporte no valido!")
            return False
    else:
        print("Parametro " + ns + "/door/apertura/picaporte/giro no especificado, valor por defecto: -pi/6")
        giro_picaporte = -np.pi / 6
    if rospy.has_param(ns + '/door/apertura/picaporte/numero_puntos'):
        numero_puntos_picaporte = rospy.get_param(ns + '/door/apertura/picaporte/numero_puntos')
    else:
        print("Parametro " + ns + "/door/apertura/picaporte/numero_puntos no especificado, valor por defecto: 10")
        numero_puntos_picaporte = 10

    if rospy.has_param(ns + '/door/apertura/puerta/giro'):
        giro_puerta = rospy.get_param(ns + '/door/apertura/puerta/giro')
    else:
        print("Parametro " + ns + "/door/apertura/puerta/giro no especificado, valor por defecto: -pi/6")
        giro_puerta = -np.pi / 6
    if rospy.has_param(ns + '/door/apertura/puerta/numero_puntos'):
        numero_puntos_puerta = rospy.get_param(ns + '/door/apertura/puerta/numero_puntos')
    else:
        print("Parametro " + ns + "/door/apertura/puerta/numero_puntos no especificado, valor por defecto: 10")
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

        resPlanTrajectory = planificarTrayectoria(
            PlanTrajectoryRequest(wp=tray.wp, type=1, planning_time=planning_time, eef_step=0.1, jump_threshold=0)
        )
        if not resPlanTrajectory.errorCode == 0:
            if resPlanTrajectory.errorCode == 1:
                print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
            else:
                return False
        if not drawDoorRViz():
            return False
        print()
        raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
        resExecuteTarget = executeTarget()
        if not resExecuteTarget.errorCode == 0:
            if resExecuteTarget.errorCode == 1:
                print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
            else:
                return False
        if not girarSistema(req_girarSistema[i]):
            return False

    # Abrir pinza
    print("Abriendo pinza...")
    GripperPubMsg.goal.command.position = OPEN_GRIPPER
    GripperPub.publish(GripperPubMsg)
    return True


if __name__ == '__main__':
    rospy.init_node("opening_door")
    rospy.wait_for_service("/arm/home_service")
    if abrirPuerta():
        print("Goal achieved!")
    else:
        print("Something was wrong :(")
    print("*** Fin del nodo /opening_door ***")
