#!/usr/bin/env python

import rospy
import sys
from OpenDoor import *
import ServicesClient
import std_srvs.srv
import armcontrolmoveit.msg

def main(ns="", test_mode=False):
    
    # Inicializar el nodo
    rospy.loginfo("Inicializando nodo...")
    rospy.init_node("open_door")
    rate = rospy.Rate(10)

    ##########################################################################################################
    # Inicio del programa:
    #       1 -> crear parametro para indicar que el brazo NO esta en posicion segura
    #       2 -> colocacion del torso
    #       3 -> colocacion del brazo en posicion segura
    #       4 -> cambiar parametro para indicar a la navegacion que puede moverse
    #       5 -> esperar a que termine la navegacion
    ##########################################################################################################

    ######### 1 ###########
    ## Comprobar la existencia del parametro de posicion segura
    if not rospy.has_param(ns + '/secure_position_arm'):
        rospy.set_param(ns + '/secure_position_arm', False)

    ######### 2 ###########
    ## Llamar al servicio que pone el torso en posicion home 
    ## para posteriormente levantarlo y tener mayor alcance
    rospy.loginfo('Asegurando posicion del torso...')
    nameService = "/rb1/torso_controllers/set_home"
    try:
        srv = rospy.ServiceProxy(nameService, std_srvs.srv.Empty)
        srv()
    except rospy.ServiceException, e:
        rospy.logwarn("Error en la llamada al servicio. Probando a publicar manualmente")
        ServicesClient.TorsoJointTrajectory()

    ######### 3 ###########
    ## Colocar el brazo en una configuracion segura #####
    rospy.loginfo('Colocando brazo en configuracion segura...')
    ## Crear peticion
    req = armcontrolmoveit.srv.SetJointValuesRequest()
    req.positions = [180, 310, 0, 320, 0, 180, 0] # Angulos en grados por comodidad
    req.positions = [ item * np.pi / 180 for item in req.positions ] # Pasar angulos a rads

    
    ## Llamada a la funcion que realiza la peticion del servicio
    res = ServicesClient.setJointValuesService(req)

    ## Respuesta == bool -> error
    ## Respuesta != bool and res.success ok -> go on
    if type(res) is bool:
        rospy.logwarn('Error en la llamada al servicio en la colocacion segura del brazo.')
        decision = raw_input('Volver a intentar? (S/N)')
        intentos = 0
        if decision.upper() == 'S':
            while type(res) is bool and not res.success and intentos < 10:
                res = ServicesClient.setJointValuesService(req)
                if type(res) is bool or not res.success:
                    intentos += 1
        
        if not i < 10 or decision.upper() == 'N':
            raw_input('Colocar el brazo manualmente en posicion segura y pulsar Enter para continuar...')
    
    ## En caso de exito, comprobar el error articular. Si es mayor de 2 grados en alguna 
    ## de las articulaciones se vuelve a invocar al servicio con la misma peticion
    if res.success:
        posok = False
        while not posok:
            error_pos = []
            for i in range(len(req.positions)):
                if abs(res.states[i]) >= 2 * np.pi - 2 * np.pi / 180:
                    error_pos.append(abs(req.positions[i] - (abs(res.states[i]) - 2 * np.pi)))
                else:
                    error_pos.append(abs(req.positions[i] - res.states[i]))

            i = 0
            while i < len(error_pos) and abs(error_pos[i]) < 2 * np.pi / 180:
                i += 1
            
            if not i == len(error_pos):
                rospy.logwarn('Posicionamiento seguro no completado. Ejecucion con demasiado error.')
                rospy.loginfo('Volviendo a invocar al servicio...')
                res = ServicesClient.setJointValuesService(req)
            else:
                rospy.loginfo('Ejecucion correcta.')
                posok = True

    ######### 4 ###########
    print('Brazo colocado en posicion segura!')
    rospy.set_param(ns + '/secure_position_arm', True)

    ######### 5 ###########
    ## Esperar que termine la fase de navegacion antes 
    ## de iniciar la apertura de la puerta
    # rospy.set_param("/secure_position_arm", True)
    i = 0
    cad = ['/', '-', '\\', '|']
    RunNav = rospy.get_param(ns + "/running_door_navigation", default=True)
    while RunNav:
        print('Esperando que termine la fase de navegacion ' + cad[i])
        sys.stdout.write("\033[F")
        i += 1
        if i >= len(cad):
            i = 0
        rate.sleep()
        RunNav = rospy.get_param(ns + "/running_door_navigation", default=True)

    ######################################################################################################
    # Inicio del proceso de apertura

    if OpenDoor(ns, test_mode):
        print("Goal achieved!")
    else:
        print("Something was wrong! :(")

#########################################################################################
#########################################################################################


if __name__ == '__main__':
    ## Lectura de parametros de entrada:
    ##  - sys.argv[0] -> nombre del programa
    ##  - sys.argv[1] -> modo test del programa o modo normal
    ##  - sys.argv[2] -> namespace de la ejecucion del nodo
    try:
        ## Lectura del namespace
        ns = ""
        if len(sys.argv) > 2:
            ns = sys.argv[2]
            if not ns[0] == '/':
                ns = '/' + ns
        ## Lectura del modo de ejecucion
        test_mode = False
        if len(sys.argv) > 1:
            test_mode = sys.argv[1] == 'test'

        ## Llamada a la funcion main
        main(ns, test_mode)
    except rospy.ROSInterruptException:
        rospy.logerr('Error en la ejecucion del nodo:' + sys.argv[0])
        
