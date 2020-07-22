#!/usr/bin/env python

import rospy
import ServicesClient
import control_msgs.msg
import trajectory_msgs.msg
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix


def OpenDoor(ns="", test_mode=False):
    # Planning time
    if rospy.has_param(ns + "move_group_config/planning_time"):
        planning_time = rospy.get_param(ns + "move_group_config/planning_time")
    else:
        planning_time = 7
    rate = rospy.Rate(1)

    # Crear variable para publicar en el topico de la pinza y del torso
    GripperPub = rospy.Publisher('/rb1/j2s7s200_gripper/gripper_command/goal',
                                 control_msgs.msg.GripperCommandActionGoal, queue_size=10)
    GripperPubMsg = control_msgs.msg.GripperCommandActionGoal()

    print("Elevando torso...")
    ServicesClient.TorsoJointTrajectory(0.15)

    ############## PRIMERA FASE: IR A POSICION HOME ##############
    raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
    resHomeSrv = ServicesClient.clienteSrvHome()
    if not resHomeSrv.errorCode == ServicesClient.SUCCESS:
        if resHomeSrv.errorCode == ServicesClient.JOINT_DIFF_ERROR:
            print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
        else:
            return False
    if not ServicesClient.drawDoorRViz():
        return False

    # Abrir la pinza al principio del codigo
    print("Abriendo pinza...")
    GripperPubMsg.goal.command.position = ServicesClient.OPEN_GRIPPER
    GripperPub.publish(GripperPubMsg)

    ############## SEGUNDA FASE: POSICIONARSE EN EL PUNTO DE APOYO ##############
    # Primero pedir posicion
    apoyo = ServicesClient.getSupport()
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
    resultGoToPoint = ServicesClient.GoToPoint(
        [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
        rpy,
        planning_time
    )
    while not resultGoToPoint == ServicesClient.SUCCESS:
        if resultGoToPoint == ServicesClient.EXECUTION_ERROR:
            print("Error al ejecutar.")
        elif resultGoToPoint == ServicesClient.PLANNING_ERROR:
            print("Error al planificar.")
        raw_input("Pulsar Enter para intentar de nuevo la planificacion y ejecucion... (ctrl+d para salir)")
        resultGoToPoint = ServicesClient.GoToPoint(
            [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
            rpy,
            planning_time
        )

    print("Movimiento realizado con exito!")

    # ################ COGER EL PICAPORTE #####################
    # Desplazarse a la posicion del picaporte y cerrar la pinza

    resultGoToPoint = ServicesClient.GoToPoint(
        [apoyo.pose.position.x, apoyo.pose.position.y, apoyo.pose.position.z],
        rpy,
        planning_time
    )
    while not resultGoToPoint == ServicesClient.SUCCESS:
        if resultGoToPoint == ServicesClient.EXECUTION_ERROR:
            print("Error al ejecutar.")
        elif resultGoToPoint == ServicesClient.PLANNING_ERROR:
            print("Error al planificar.")
        raw_input("Pulsar Enter para intentar de nuevo la planificacion y ejecucion... (ctrl+d para salir)")
        resultGoToPoint = ServicesClient.GoToPoint(
            [apoyo.pose.position.x, apoyo.pose.position.y, apoyo.pose.position.z],
            rpy,
            planning_time
        )

    print("Movimiento realizado con exito!")

    # Cerrar pinza
    print("Cerrando pinza...")
    GripperPubMsg.goal.command.position = ServicesClient.CLOSE_GRIPPER
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

    ###################### Girar el picaporte ######################
    ## Si se ejecuta en modo test, la trayectoria de abrir la puerta se divide en n puntos,
    ## de esta manera se observa a partir de que angulo girado empieza a haber problemas.
    ## Si se ejecuta en modo normal, solo se planifica una unica trayectoria con el maximo
    ## angulo de giro que se especifique en los parametros de ROS.
    if test_mode:
        ## TEST
        req_generarTrayectoria = [ServicesClient.GenerateDoorTrajectoryRequest(giro_picaporte, numero_puntos_picaporte, sistemaPicaporte)]
        req_girarSistema = [ServicesClient.RotateSystemRequest(giro_picaporte, sistemaPicaporte)]
        for i in range(numero_puntos_puerta):
            req_generarTrayectoria.append(ServicesClient.GenerateDoorTrajectoryRequest(giro_puerta/numero_puntos_puerta, 1, sistemaPuerta))
            req_girarSistema.append(ServicesClient.RotateSystemRequest(giro_puerta/numero_puntos_puerta, sistemaPuerta))
        req_generarTrayectoria.append(ServicesClient.GenerateDoorTrajectoryRequest(-giro_picaporte, numero_puntos_picaporte, sistemaPicaporte))
        req_girarSistema.append(ServicesClient.RotateSystemRequest(-giro_picaporte, sistemaPicaporte))
    else:
        ## NORMAL
        req_generarTrayectoria = [ServicesClient.GenerateDoorTrajectoryRequest(giro_picaporte, numero_puntos_picaporte, 1),
                                ServicesClient.GenerateDoorTrajectoryRequest(giro_puerta, numero_puntos_puerta, 0),
                                ServicesClient.GenerateDoorTrajectoryRequest(-giro_picaporte, numero_puntos_picaporte, 1)]
        req_girarSistema = [ServicesClient.RotateSystemRequest(giro_picaporte, 1),
                            ServicesClient.RotateSystemRequest(giro_puerta, 0),
                            ServicesClient.RotateSystemRequest(-giro_picaporte, 1)]

    ###################################################################################################################
    # Se han guardado todas los puntos de paso en arrays
    # para ahora recorrerlos con un for y poder hacer la planificacion secuencialmente.
    # Dentro se ejecutara un bucle while que no saldra hasta que se haya cumplido con exito la ejecucion, planificando
    # una y otra vez hasta que se cumpla el objetivo o el usuario salga del programa.
    ###################################################################################################################
    for i in range(len(req_generarTrayectoria)):
        # Se calculan los puntos de paso de las trayectorias circulares
        tray = ServicesClient.generateTrajectory(req_generarTrayectoria[i])
        if type(tray) is bool and not tray:
            return False

        # Put an infinite loop in your life
        success = False
        while not success:
            # Planning
            resPlanTrajectory = ServicesClient.planTrajectory(
                PlanTrajectoryRequest(wp=tray.wp, type=1, planning_time=planning_time, eef_step=0.1, jump_threshold=0)
            )
            # Check planning
            if not resPlanTrajectory.errorCode == ServicesClient.SUCCESS:
                if resPlanTrajectory.errorCode == ServicesClient.JOINT_DIFF_ERROR:
                    print("Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
                    if not ServicesClient.drawDoorRViz():
                        print("Fallo al dibujar la puerta en RViz. Reintentando...")
                        ServicesClient.drawDoorRViz()
                    print()
                else:
                    print("Error en la planificacion. Volviendo a planificar la trayectoria...")

            if resPlanTrajectory.errorCode == ServicesClient.SUCCESS or resPlanTrajectory.errorCode == ServicesClient.JOINT_DIFF_ERROR:
                # Planning was good, to execute
                raw_input("Pulsar Enter para continuar (crtl+d para salir)...")
                resExecuteTarget = ServicesClient.executeTarget()
                # Check execution
                if not resExecuteTarget.errorCode == ServicesClient.SUCCESS:
                    if resExecuteTarget.errorCode == ServicesClient.JOINT_DIFF_ERROR:
                        print(
                            "Warning! se ha alterado el punto inicial del plan para que coincida con la posicion actual.")
                        success = True
                    else:
                        print("Error en la ejecucion. Volviendo a planificar la trayectoria...")
                else:
                    success = True

        if not ServicesClient.rotateSystem(req_girarSistema[i]):
            return False

    # Abrir pinza
    print("Abriendo pinza...")
    GripperPubMsg.goal.command.position = ServicesClient.OPEN_GRIPPER
    GripperPub.publish(GripperPubMsg)

    # Pedir posicion de apoyo para retirar el brazo
    apoyo = ServicesClient.getSupport()
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

    # Retirar la mano un poco del picaporte para evitar colisiones
    resultGoToPoint = ServicesClient.GoToPoint(
        [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
        rpy,
        planning_time
    )
    while not resultGoToPoint == ServicesClient.SUCCESS:
        if resultGoToPoint == ServicesClient.EXECUTION_ERROR:
            print("Error al ejecutar.")
        elif resultGoToPoint == ServicesClient.PLANNING_ERROR:
            print("Error al planificar.")
        raw_input("Pulsar Enter para intentar de nuevo la planificacion y ejecucion... (ctrl+d para salir)")
        resultGoToPoint = ServicesClient.GoToPoint(
            [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
            rpy,
            planning_time
        )

    print("Movimiento realizado con exito!")

    # Retirar la pinza del plano de la puerta para poder ir a la posicion home sin problemas
    pos_prep = np.dot(pos_prep, np.array([[1, 0, 0, 0],
                                          [0, 1, 0, -0.2],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]]))

    # Realizar la planificacion del punto al que se desea ir
    resultGoToPoint = ServicesClient.GoToPoint(
        [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
        rpy,
        planning_time
    )
    while not resultGoToPoint == ServicesClient.SUCCESS:
        if resultGoToPoint == ServicesClient.EXECUTION_ERROR:
            print("Error al ejecutar.")
        elif resultGoToPoint == ServicesClient.PLANNING_ERROR:
            print("Error al planificar.")
        raw_input("Pulsar Enter para intentar de nuevo la planificacion y ejecucion... (ctrl+d para salir)")
        resultGoToPoint = ServicesClient.GoToPoint(
            [pos_prep[0, 3], pos_prep[1, 3], pos_prep[2, 3]],
            rpy,
            planning_time
        )

    print("Movimiento realizado con exito!")

    # Realizar la planificacion del punto al que se desea ir
    resultGoToPoint = ServicesClient.GoToPoint(
        [0.5, 0, 0.7],
        [0, 0, 0],
        planning_time
    )
    while not resultGoToPoint == ServicesClient.SUCCESS:
        if resultGoToPoint == ServicesClient.EXECUTION_ERROR:
            print("Error al ejecutar.")
        elif resultGoToPoint == ServicesClient.PLANNING_ERROR:
            print("Error al planificar.")
        raw_input("Pulsar Enter para intentar de nuevo la planificacion y ejecucion... (ctrl+d para salir)")
        resultGoToPoint = ServicesClient.GoToPoint(
            [0.5, 0, 0.7],
            [0, 0, 0],
            planning_time
        )

    print("Movimiento realizado con exito!")

    return True

##########################################################################################################

