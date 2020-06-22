/*
* Door.cpp
*/

#include <armcontrolmoveit/Door.h>

Door::Door(geometry_msgs::Point apoyo, geometry_msgs::Point ejepicaporte, geometry_msgs::Point ejepuerta)
{
    this->vstool = new VisualTools();
    /* 
     * Haciendo los calculos de la distancia mas corta entre la recta formada por el punto "ejepuerta"
     * y el vector apoyo-ejepicaporte con el punto eje-picaporte, se puede sacar el punto intermedio 
     * para sacar todas las medidas necesarias de la puerta en funcion de estos 3 puntos. Se asume que
     * los 3 puntos se encuentran en el mismo plano Z
     *      B               Q
     *      ·---------------·
     *                      |
     *               A ·----· P
    */
    
    geometry_msgs::Point A = apoyo, B = ejepuerta, P = ejepicaporte, Q; /* Para abreviar */
    float alpha = M_PI_2;
    if (ejepicaporte.x-apoyo.x != 0)
    {
        float m = (ejepicaporte.y-apoyo.y)/(ejepicaporte.x-apoyo.x);
        alpha = atan(m);
        Q.x = (P.x-m*(B.y-m*B.x-P.y))/(pow(m, 2)+1);
        Q.y = B.y + m*(Q.x-B.x);
        Q.z = A.z;
    } else {
        Q.x = ejepuerta.x;
        Q.y = ejepicaporte.y;
        Q.z = apoyo.z;
        if (ejepicaporte.y-apoyo.y < 0)
            alpha = -M_PI_2;
    }
    

    std::cout << "Puerta: [" << B.x << ", " << B.y << ", " << B.z << "]\n";
    std::cout << "Picaporte1: [" << Q.x << ", " << Q.y << ", " << Q.z << "]\n";
    std::cout << "Picaporte2: [" << P.x << ", " << P.y << ", " << P.z << "]\n";
    std::cout << "Apoyo: [" << A.x << ", " << A.y << ", " << A.z << "]\n\n";

    /* Calculo de las medidas entre los puntos obtenidos */
    this->width_latch = distancia_euclidea(A, P);
    this->depth_latch = distancia_euclidea(P, Q);
    this->width_door = distancia_euclidea(Q, B);
    this->height_door = B.z;
    std::cout << "width_latch: " << width_latch << " m\n";
    std::cout << "depth_latch: " << depth_latch <<" m\n";
    std::cout << "width_door: " << width_door<<" m\n";
    std::cout << "height_door: " << height_door << " m\n\n";

    /* Se generan los mensajes de posicion y orientacion y se guardan como atributos de la clase */
    Affine3d aux;
    tf::Quaternion q;

    /* La orientacion del eje de la puerta va a tener el eje Z sobre el eje de giro y el eje X sobre
     * el plano de la puerta. El giro sobre Z vendra dado por el arcotangente de la pendiente del vector
     * que define el plano, en este caso podroa ser AP. Ya se ha calculado anteriormente su pendiente (m) 
     * */

    aux = Affine3d::Identity();
    aux = Translation3d(Vector3d(ejepuerta.x, ejepuerta.y, ejepuerta.z))*AngleAxisd(alpha, Vector3d::UnitZ());
    
    this->puerta.push_back(aux);
    this->angulo_actual.push_back(0);

    

    /* La orientacion del resto de ejes será con el eje Z perpendicular al plano de la puerta y el eje X 
     * paralelo al plano. Ideal usar angulos de Euler ZYZ para este paso */
    aux *= Translation3d(Vector3d(this->width_door, 0, 0));
    aux *= AngleAxisd(M_PI_2, Vector3d::UnitZ())*AngleAxisd(M_PI_2, Vector3d::UnitY())*AngleAxisd(M_PI_2, Vector3d::UnitZ());
    this->puerta.push_back(aux);
    this->angulo_actual.push_back(0);
    

    /* La orientacion del punto de apoyo debe ser las misma que para el eje de giro del picaporte al ser sistemas
     * solidarios. Solo se aplica traslacion. */
    aux *= Translation3d(Vector3d(0, 0, -this->depth_latch));
    this->puerta.push_back(aux);
    this->angulo_actual.push_back(0);
    
    aux *= Translation3d(Vector3d(this->width_latch, 0, 0));
    this->puerta.push_back(aux);
    this->angulo_actual.push_back(0);
    
    this->drawInRViz();
}

Door::~Door() {}

void Door::drawInRViz()
{
    // this->vstool->deleteAllMarkers();
    this->vstool->drawDoor(*this);
}

VisualTools* Door::getvstool()
{
    return this->vstool;
}

float Door::getWidthDoor() { return this->width_door; }
float Door::getHeightDoor() { return this->height_door; }
float Door::getDepthLatch() { return this->depth_latch; }
float Door::getWidthLatch() { return this->width_latch; }

geometry_msgs::Pose Door::getSystemPose(int index)
{
    geometry_msgs::Pose p;
    transform2pose(this->puerta.at(index), p);
    return p;
}

geometry_msgs::Point Door::getSystemPoint(int index)
{
    geometry_msgs::Pose p;
    transform2pose(this->puerta.at(index), p);
    return p.position;
}

float Door::getSystemAngle(int index)
{
    return this->angulo_actual.at(index);
}

void Door::normalizarAngulo(float &angulo)
{
    while (angulo <= -2*M_PI || angulo >=2*M_PI)
    {
        if (angulo < -2*M_PI)
        {
            angulo += 2*M_PI;
        } else angulo -= 2*M_PI;
    }
}

/* *********************** Services *********************** */

bool Door::generarTrayectoria(armcontrolmoveit::GenerarTrayectoriaPuertaRequest &req, armcontrolmoveit::GenerarTrayectoriaPuertaResponse &res)
{
    bool success = false;
    this->normalizarAngulo(req.angulo);

    if (req.angulo != 0)
    {
        if (req.sistema == sistemas::EJEPUERTA)
        {
            /* Bucle para generar los distintos puntos de la trayectoria */
            geometry_msgs::Pose p;
            Affine3d apoyo, tfpuerta2apoyo;
            tfpuerta2apoyo = Translation3d(Vector3d(this->width_door, 0, 0))*AngleAxisd(M_PI_2, Vector3d::UnitZ())*
                    AngleAxisd(M_PI_2, Vector3d::UnitY())*AngleAxisd(M_PI_2, Vector3d::UnitZ())*
                    Translation3d(Vector3d(0, 0, -this->depth_latch))*
                    AngleAxisd(this->angulo_actual.at(sistemas::EJEPICAPORTE2), Vector3d::UnitZ())*
                    Translation3d(Vector3d(this->width_latch, 0, 0));
            for (int punto = 0; punto <= (int)req.np; punto++)
            {
                apoyo = this->puerta.at(sistemas::EJEPUERTA)*AngleAxisd(req.angulo*punto/req.np, Vector3d::UnitZ())*tfpuerta2apoyo;
                transform2pose(apoyo, p);
                res.wp.push_back(p);
                std::cout << "Punto calculado #" << punto << ": [" << p.position.x << ", " << p.position.y << ", " << p.position.z << "] [";
                std::cout << p.orientation.w << ", " << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << "]\n";
            }
        } else {
            
            /* Bucle para generar los distintos puntos de la trayectoria */
            geometry_msgs::Pose p;
            Affine3d apoyo;
            std::cout << "N" << req.np << std::endl;
            for (int punto = 0; punto <= req.np; punto++)
            {
                std::cout << "N: " <<punto << "/"<<  req.np << std::endl;
                apoyo = this->puerta.at(sistemas::EJEPICAPORTE2)*
                        AngleAxisd(punto*req.angulo/req.np, Vector3d::UnitZ()) * Translation3d(Vector3d(this->width_latch, 0, 0)); 
                transform2pose(apoyo, p);
                res.wp.push_back(p);
                std::cout << "Punto calculado #" << punto << ": [" << p.position.x << ", " << p.position.y << ", " << p.position.z << "] [";
                std::cout << p.orientation.w << ", " << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << "]\n";
            }
            
            
        }
        success = true;
    } else ROS_ERROR("Angulo nulo introducido, ninguna accion realizada.");
    return success;
}

bool Door::girarSistema(armcontrolmoveit::GirarSistemaPuertaRequest &req, armcontrolmoveit::GirarSistemaPuertaResponse &res)
{
    bool success = false;
    this->normalizarAngulo(req.angulo);
    if (req.angulo != 0)
    {
        if (req.sistema == sistemas::EJEPUERTA)
        {
            this->angulo_actual.at(sistemas::EJEPUERTA) += req.angulo;
            this->puerta.at(sistemas::EJEPUERTA) *= AngleAxisd(req.angulo, Vector3d::UnitZ());
            this->puerta.at(sistemas::EJEPICAPORTE1) = this->puerta.at(sistemas::EJEPUERTA)*Translation3d(Vector3d(this->width_door, 0, 0))*
                    AngleAxisd(M_PI_2, Vector3d::UnitZ())*AngleAxisd(M_PI_2, Vector3d::UnitY())*AngleAxisd(M_PI_2, Vector3d::UnitZ());
            this->puerta.at(sistemas::EJEPICAPORTE2) = this->puerta.at(sistemas::EJEPICAPORTE1)*Translation3d(Vector3d(0, 0, -this->depth_latch));
            this->puerta.at(sistemas::EJEPICAPORTE2) *= AngleAxisd(this->angulo_actual.at(sistemas::EJEPICAPORTE2), Vector3d::UnitZ());
            this->puerta.at(sistemas::APOYO) = this->puerta.at(sistemas::EJEPICAPORTE2)*Translation3d(Vector3d(this->width_latch, 0, 0));
        } else {
            this->angulo_actual.at(sistemas::EJEPICAPORTE2) += req.angulo;
            this->puerta.at(sistemas::EJEPICAPORTE2) *= AngleAxisd(req.angulo, Vector3d::UnitZ());
            this->puerta.at(sistemas::APOYO) = this->puerta.at(sistemas::EJEPICAPORTE2)*Translation3d(Vector3d(this->width_latch, 0, 0));
        }
        this->drawInRViz();
        success = true;
    } else ROS_ERROR("Angulo nulo introducido, ninguna accion realizada.");
    return success;
}

bool Door::posicionApoyo(armcontrolmoveit::PosicionApoyoRequest &req, armcontrolmoveit::PosicionApoyoResponse &res)
{
    res.pose = this->getSystemPose(sistemas::APOYO);
    return true;
}

bool Door::drawDoorRViz(armcontrolmoveit::DrawDoorRVizRequest &req, armcontrolmoveit::DrawDoorRVizResponse &res)
{
    this->drawInRViz();
    return true;
}

/* *********************** Operadores *********************** */

Affine3d Door::operator[](int index)
{
    return this->puerta.at(index);
}

