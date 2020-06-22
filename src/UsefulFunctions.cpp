/*

    UsefulFunctions.cpp

*/

#include <armcontrolmoveit/UsefulFunctions.h>

// template <typename S, typename T>
// std::string myStrCat(S str1, T str2)
// {
//     /* 
//      * Funcion para concatenar dos char* 
//      * en un string, para el constructor 
//      * donde se especifica un namespace 
//     */
//     std::string strout(str1);
//     int i = 0;
//     while (strcmp(&str2[i], "\0"))
//     {
//         strout.push_back(str2[i]);
//         i++;
//     }
//     return strout;
// }

float distancia_euclidea(geometry_msgs::Point p, geometry_msgs::Point q)
{
    return sqrt(pow(p.x-q.x, 2) + pow(p.y-q.y, 2) + pow(p.z-q.z, 2));
}

void transform2pose(Eigen::Affine3d t, geometry_msgs::Pose &pose)
{
	/* Variables necesarias para extraer la rotacion en quaternios de la matriz t*/
	tf::Matrix3x3 rot;
	tf::Quaternion _q;

	for (int fil = 0; fil < 3; fil++)
	{
		for (int col = 0; col < 3; col++)
		{
			rot[fil][col] = t(fil, col);
		}
	}
	rot.getRotation(_q);
	_q.normalize();

	/* Se introducen los datos en el mensaje de posicion */
	pose.position.x = t.matrix()(0, 3);
	pose.position.y = t.matrix()(1, 3);
	pose.position.z = t.matrix()(2, 3);
	pose.orientation.x = _q.x();
	pose.orientation.y = _q.y();
	pose.orientation.z = _q.z();
	pose.orientation.w = _q.w();
}