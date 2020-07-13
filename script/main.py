#!/usr/bin/env python

from OpenDoor import *


if __name__ == '__main__':
    print("Inicializando nodo...")
    # Inicializar el nodo
    rospy.init_node("open_door")
    # Esperar a los servicios de las clases necesarios
    print("Esperando a los servicios de Door...")
    rospy.wait_for_service("/door/posicion_apoyo")
    print("Esperando a los servicios de ArmControl...")
    rospy.wait_for_service("/arm/home_service")

    # Comprobar el namespace
    ns = ""
    if len(sys.argv) > 2:
        ns = sys.argv[2]
        if not ns[-1] == '/':
            ns = ns + '/'
        if not ns[0] == '/':
            ns = '/' + ns
    success = False
    if sys.argv[1] == "test":
        print("Ejecutando modo test...")
        success = TestOpenDoor(ns)
    else:
        print("Ejecutando modo normal...")
        success = OpenDoor(ns)

    if success:
        print("Goal achieved!")
    else:
        print("Something was wrong! :(")
