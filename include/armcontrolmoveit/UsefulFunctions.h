/* 

    UsefulFunctions.h

    Funciones varias que no sabia donde meter y 
    eran suficientes para hacer un archivo a parte :)

*/

#ifndef USEFULFUNCTIONS_H
#define USEFULFUNCTIONS_H

#include <iostream>
#include <string>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

/* Funcion para concatenar dos strings o char* o const char* 
 * para la parte de ejecutar los nodos dentro de un namespace.
 * Se hace un template<class S, class T> para que puedan ser de 
 * distinta clase las entradas.
 *  - Input:
 *      str1 -> cadena que queda como la primera parte de la salida
 *      str2 -> cadena que queda como la segunda parte de la salida
 *  - Output:
 *      string -> cadena con las 2 entradas concatenadas
 */
template <typename S, typename T>
std::string myStrCat(S, T);

/* Se programa la funcion aqui porque sino da error al compilar
 por culpa del template T.T */
template <typename S, typename T>
std::string myStrCat(S str1, T str2)
{
    /* 
     * Funcion para concatenar dos char* 
     * en un string, para el constructor 
     * donde se especifica un namespace 
    */
    std::string strout(str1);
    int i = 0;
    while (strcmp(&str2[i], "\0"))
    {
        strout.push_back(str2[i]);
        i++;
    }
    return strout;
}

/* Calculo de la distancia euclidea entre dos puntos, 
 * para los calculos de la puerta.
 *  - Input:
 *      geometry_msgs::point -> puntos en coordenadas cartesianas
 *  - Output:
 *      float -> distancia euclidea entre los puntos
 */
float distancia_euclidea(geometry_msgs::Point, geometry_msgs::Point);

/* Transformar de un objeto Affine3d que es una 
 * matriz de transformacion homogenea, al objeto Pose con posicion y orientacion 
 */
void transform2pose(Eigen::Affine3d, geometry_msgs::Pose &);

/* Funcion valor absoluto. 
You already know how this works */
template <typename T>
T myabs(T value);

template <typename T>
T myabs(T value)
{
    if (value < 0)
        return -value;
    return value;   
}


/* Funcion para dejar todos los angulos entre 0 y 2pi */
template <typename T>
T normalizeAngle(T angle);

template <typename T>
T normalizeAngle(T angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
    return angle;
}


#endif