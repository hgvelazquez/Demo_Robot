/* Visualization tool to see the lines drawn by simulated sensors. */
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Sensor.h"
#include "MapFiller.h"

#define _USE_MATH_DEFINES

geometry_msgs::Pose KOBUKI_POSITION;
nav_msgs::OccupancyGrid MAP;
int nav_move;
geometry_msgs::Twist current_vel;
geometry_msgs::Twist nav_vel;
geometry_msgs::Point nav_goal;
bool turn;
bool have_map;
float XI = 0.8;
float ETA = 1.2;
bool turning; // Agregamos esta bandera para que gire hasta que no se oriente al goal (aprox).
// Hay que agregar la parte paera
bool changed_goal;
const int SENSOR_NUMBER = 18; // Número de sensores que se dibujarán a la kobuki
Sensor SENSORS[SENSOR_NUMBER]; // Arreglo con los sensores
std::string s;

/**
 * Función que se encarga de crear los sensores que tendrá 
 * la simulación. Pone cada uno de los sensores sobre 
 * la circunferencia de la KOBUKI de manera equidistate.
 * Inicializa el arreglo de sensores Sensor SENSOR_NUMBER.
 */
void putSensors() 
{
    // Obtenemos el tamaño del ángulo entre los sensores. 
    float fraction = 2*M_PI/SENSOR_NUMBER;
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, fraction);
    rotation.normalize();
    // Vamos a rotar este vector sucesivamente por el ángulo dado por la fracción.
    tf2::Quaternion v(1, 0, 0, 0);
    // Ponemos los sensores.
    for (int i = 0; i<SENSOR_NUMBER; i++) { 
        Sensor sensor;
        sensor.id = i;
        sensor.angle_vector = v;
        // Obtenemos el ángulo relativo a la KOBUKI.
        sensor.angle = i*fraction;
        // Consideramos estos sensores con mayor peso en el movimiento. 
        sensor.front = (i*fraction <= M_PI/4 || i*fraction >= 7*M_PI/4);
        // Rotamos el vector una vez más por el ángulo. 
        v = rotation*v*inverse(rotation);
        v.normalize();
        SENSORS[i] = sensor;
    }  
} 

/**
 * Función callback para recibir las metas de la navegación.
 */
void nav_receiveNavGoal(const geometry_msgs::Point& goalStamped)
{
    changed_goal = (nav_goal.x != goalStamped.x || nav_goal.y != goalStamped.y || nav_move == 0);
    nav_goal = goalStamped;      
    nav_move = 1;
}

/**
 * Función callback para recibir la información del mapa.
 * En el momento en el que recibamos un mapa válido (checamos que
 * su resolución no sea 0), marcamos have_map como verdadero y 
 * no escucharemos más (asumimos que el mapa no cambia).
 */
void getMapParams(const  nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    MAP = *msg; 
    have_map = MAP.info.resolution != 0;
}

/**
 * Función callback para actualizar la información necesaria 
 * para los sensores, incluyendo la posición y orientación 
 * actual de la KOBUKI.
 */
void updateSensors(const nav_msgs::Odometry::ConstPtr& msg) 
{
    geometry_msgs::Pose pose;
    nav_msgs::Odometry od = *msg;
    pose = od.pose.pose;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    q.normalize();
    // Actualizamos la información de los sensores. 
    for (int i = 0; i < SENSOR_NUMBER; i++) 
    {
        // Rotamos el ángulo respecto a la KOBUKI con la orientación real.
        tf2::Quaternion new_angle = q*SENSORS[i].angle_vector*inverse(q);
        new_angle.normalize();
        SENSORS[i].absolute_angle = new_angle;
        SENSORS[i].current_position = pose.position;
    }
    // Actualizamos a posición y orientación de la KOBUKI.
    KOBUKI_POSITION = pose;
    // Si estamos lo suficientemente cerca de la meta, marcamos que ya no hay que moverse.
    if (SENSORS[0].distance(pose.position, nav_goal) < 0.1)
        nav_move = 0;  
}

/**
 * Función que se encarga del cálculo de la velocidad dada la información de
 * los sensores. Como siempre se llama después de drawSensors, asumimos
 * que cada uno de los sensores ya calculó su punto de colisión. 
 */
geometry_msgs::Twist navigate() {
    geometry_msgs::Twist nav_new;
    geometry_msgs::Pose point = KOBUKI_POSITION;
    tf2::Quaternion v1(point.orientation.x, point.orientation.y, point.orientation.z, point.orientation.w);
    geometry_msgs::Point ZERO;
    
    // Pasamos el goal al marco de la KOBUKI.
    geometry_msgs::Point nav_goal_coord;
    // Trasladamos.
    nav_goal_coord.x = nav_goal.x -  point.position.x;
    nav_goal_coord.y = nav_goal.y -  point.position.y;
    // Rotamos.
    tf2::Quaternion g(nav_goal_coord.x, nav_goal_coord.y, 0, 0);
    g = inverse(v1)*g*v1;
    nav_goal_coord.x = g.x();
    nav_goal_coord.y = g.y();
    
    if (turning == true && !changed_goal) {
        tf2::Vector3 v1(nav_goal_coord.x, nav_goal_coord.y, 0);
        tf2::Vector3 v2(1, 0, 0);
        float a = v2.angle(v1);
        if (a > M_PI/20) {
            float factor = (a < M_PI/3) ? 1.5 : 1; 
            turning = true;
            nav_new.linear.x = 0;
            int sign = (nav_goal_coord.y > 0) ? 1 : -1;
            nav_new.angular.z = factor*sign*v2.angle(v1);
            turn = true;
            return nav_new;
        } else {
            turning = false;
        }
    }
    
    if (nav_move == 1) {
        float min = 1;
        int min_sensor = -1;
        float nav_d0 = 0.9;
        for(int i = 0; i < SENSOR_NUMBER; i++) {
    
            float angle = SENSORS[i].angle;    
          
            geometry_msgs::Point actual;
            
            geometry_msgs::Point end;

            end.x = SENSORS[i].distance_detected*cos(angle); 
            end.y = SENSORS[i].distance_detected*sin(angle);
            // Frontera a partir de la cual contamos las colisiones.
            // Será el punto puesto en la orilla de la KOBUKI. (rcost, rsent)
            actual.x = RESOLUTION*cos(SENSORS[i].angle); 
            actual.y = RESOLUTION*sin(SENSORS[i].angle);
          
            // Posible error por el 0.2 en lugar de resolution
            // aunque no sé a que se deba ese valor
            float nav_d = SENSORS[i].distance_detected - 0.2; // Movemos los sensores a la orilla de la KOBUKI.
            if (nav_d < min && SENSORS[i].front) {
                min = nav_d;
                min_sensor = i;
            }    
            // El peso que tendrá un sensor depende de su ángulo. 
            float weight = 2*fabs(M_PI - angle)/6;
            if (nav_d < nav_d0) {
                nav_d = (nav_d < 0.00001) ? 0.00001 : nav_d; // Tope a la fuerza repulsora.
                // Aumentamos la fuerza repulsiva si estamos muy cerca.
                if (nav_d < 0.3 && SENSORS[i].front) 
                    nav_d *= 0.01;
                else if (nav_d < 0.4) 
                    nav_d *= 0.01; 
            
                float factor = ETA*((1/nav_d) - (1/nav_d0))*(1/nav_d*nav_d*nav_d);  
            
                nav_new.linear.x += weight*factor*(-end.x);
                nav_new.angular.z += weight*factor*(-end.y);
            }  
        }
      
        float nav_d = SENSORS[0].distance(ZERO, nav_goal_coord);
        float deltax = -nav_goal_coord.x;
        float deltay = - nav_goal_coord.y;
        
        if (nav_d > 2) {
            nav_new.linear.x -= 0.5*XI*deltax;
            nav_new.angular.z -= 0.5*XI*deltay;
        }
        else if (nav_d < 0.4) {
            nav_new.linear.x -= SENSOR_NUMBER*10*XI*deltax;
            nav_new.angular.z -= SENSOR_NUMBER*10*XI*deltay;
            float norm = sqrt(nav_new.linear.x*nav_new.linear.x + nav_new.angular.z*nav_new.angular.z);
            nav_vel.linear.x = nav_new.linear.x/norm;
            nav_vel.angular.z = nav_new.angular.z/norm;
            return nav_vel;
        }else {  
            nav_new.linear.x -= 2*XI*deltax;
            nav_new.angular.z -= 2*XI*deltay;
        }
      
        if (!changed_goal) {
            float sumx = 0.7*nav_vel.linear.x + 0.3*nav_new.linear.x;
            float sumz = 0.7*nav_vel.angular.z + 0.3*nav_new.angular.z;
            float norm = sumx*sumx + sumz*sumz;
            norm = sqrt(norm);
            nav_vel.linear.x = sumx/(norm);
            nav_vel.angular.z = sumz/(norm);
        } else {
            tf2::Vector3 v1(nav_goal_coord.x, nav_goal_coord.y, 0);
            tf2::Vector3 v2(1, 0, 0);
            if (v2.angle(v1) > M_PI/9) {
                turning = true;
                turn = true;
            }
            float norm = sqrt(nav_new.linear.x*nav_new.linear.x + nav_new.angular.z*nav_new.angular.z);
            nav_vel.linear.x = nav_new.linear.x/(norm);
            nav_vel.angular.z = nav_new.angular.x/(norm);  
            changed_goal = false;  
        } 
        
        if (min_sensor != -1 && min < 0.28) {
            float a = SENSORS[min_sensor].angle;
            int sign = (a > M_PI) ? -1 : 1; 
            nav_vel.angular.z *= sign*2;
            float norm = sqrt(nav_vel.linear.x*nav_vel.linear.x + nav_vel.angular.z*nav_vel.angular.z);
            nav_vel.linear.x = nav_vel.linear.x/(norm);
            nav_vel.angular.z = nav_vel.angular.x/(norm);  
            
        }
        
        if (nav_vel.linear.x < 0){
            tf2::Vector3 v1(nav_goal_coord.x, nav_goal_coord.y, 0);
            tf2::Vector3 v2(1, 0, 0);
            if (v2.angle(v1) > M_PI/9) {
                turning = true;
                nav_vel.linear.x = 0;
                int sign = (nav_goal_coord.y > 0) ? 1 : -1;
                nav_vel.angular.z = sign*v2.angle(v1);
                turn = true;   
            } else {
                nav_vel.linear.x = -XI*deltax;
                nav_vel.angular.x = -XI*deltay;
            }
        }
    }
    else {
        nav_vel.linear.x = 0;
        nav_vel.angular.z = 0;
    }
    current_vel = nav_vel;
    return nav_vel;
}

/**
 * Función que se encarga de dibujar la flecha con la velocidad 
 * vel que reciba como parámetro. 
 */
visualization_msgs::Marker drawSpeed(geometry_msgs::Twist vel) 
{   
    // El punto origen en coordenadas de la KOBUKI.
    geometry_msgs::Point ZERO;
    
    visualization_msgs::Marker nav_sensor;
    //General data of the Vector
    nav_sensor.id = 100; // Más que cualquier número de sensores.
    nav_sensor.header.frame_id = ("/"+s+"_tf/base_link").c_str();
    nav_sensor.header.stamp = ros::Time::now();   // No caduca
    nav_sensor.ns = "vel";
    nav_sensor.type = visualization_msgs::Marker::ARROW;
    nav_sensor.action = visualization_msgs::Marker::ADD;
    
    nav_sensor.pose.orientation.w = 1; // All the others are zero.
    
    nav_sensor.scale.x = 0.05;
    nav_sensor.scale.y = 0.1;
    nav_sensor.scale.z = 0.1;
    
    nav_sensor.color.a = 1.0; // Same transparency for all. 
    nav_sensor.color.b = 1.0;
    
    // We put the start point and end point.
    nav_sensor.points.push_back(ZERO);
    
    geometry_msgs::Point end;
    end.x = vel.linear.x; 
    end.y = vel.angular.z;
    // End point
    nav_sensor.points.push_back(end);
    
    return nav_sensor;
}

/**
 * Función que pinta los sensores en el mapa. 
 * No recibe parámetros, lo único que necesita es el 
 * ángulo de cada sensor y la posición actual de la KOBUKI.
 */
visualization_msgs::MarkerArray drawSensors()
{
    visualization_msgs::MarkerArray sensors;
    // Origen en coordenadas de la KOBUKI.
    geometry_msgs::Point ZERO;
  
    for(int i = 0; i < SENSOR_NUMBER; i++) 
    {
        visualization_msgs::Marker current_sensor;

        //General data of the Line
        if (s == ""){
            current_sensor.header.frame_id = "/base_link"; // En coordenadas de la KOBUKI.
            //ROS_ERROR("CamposSensores.cpp: current sensor : /base_link");
        }else
        {
            current_sensor.header.frame_id = s+"_tf/base_link"; // En coordenadas de la KOBUKI.
            //ROS_ERROR("CamposSensores.cpp: current sensor : %s", ("/"+s+"_tf/base_link").c_str());   
        }
        
        current_sensor.id = i;
        current_sensor.header.stamp = ros::Time::now();   // No caduca
        current_sensor.ns = "sensor";  
        current_sensor.type = visualization_msgs::Marker::LINE_LIST;
        current_sensor.action = visualization_msgs::Marker::ADD;
        current_sensor.pose.orientation.w = 1; // All the others are zero.
        
        current_sensor.scale.x = 0.02;
        current_sensor.color.a = 1.0; // Same transparency for all. 
    
        // We put the start point and end point.
        current_sensor.points.push_back(ZERO);
        // Encontramos la colisión.
        SENSORS[i].getEndPoint(MAP);
        // Pintamos con el ángulo y la distancia detectada.
        geometry_msgs::Point end;
        float angle = SENSORS[i].angle;
        end.x = SENSORS[i].distance_detected*cos(angle); 
        end.y = SENSORS[i].distance_detected*sin(angle);
        // We put the end point.
        current_sensor.points.push_back(end);
    
        // If it collides, we paint it red.
        if (SENSORS[i].collision) {
            current_sensor.color.g = 0.0;
            current_sensor.color.r = 1.0;
        } // Else, green
        else {  
            current_sensor.color.g = 1.0;
            current_sensor.color.r = 0.0;
        } 
    
        sensors.markers.push_back(current_sensor);
    }
    return sensors;
}  

// Función main
int main (int argc, char** argv)
{
    // Iniciamos los componentes de ROS  
    ros::init(argc, argv, "collisions");
    ros::NodeHandle n("~");
    ros::Rate r(20);
    
    if (!n.hasParam("robot_name"))
    {
        ROS_ERROR("CamposSensores.cpp: No param named 'robot_name'");
        s = "";
    }

    if (n.getParam("robot_name", s))
    {
      ROS_ERROR("CamposSensores.cpp: Got param %s", s.c_str());
    }
    else
    {
      ROS_ERROR("CamposSensores.cpp: Failed to get param 'robot_name'");
      s = "";
    }

  
    // Los tópicos donde publicaremos.
    ros::Publisher sensor_marker = n.advertise<visualization_msgs::MarkerArray>("sensor_markers", 10);
    ros::Publisher vector_marker = n.advertise<visualization_msgs::Marker>("vector_marker", 10);
    ros::Publisher nav_velocity_pub = n.advertise<geometry_msgs::Twist>("/HOLA/mobile_base/commands/velocity", 1);
    
    ros::Subscriber sub_odom = n.subscribe(("/"+s+"/odom").c_str(), 5, updateSensors);
    ROS_INFO("CamposSensores.cpp: suscribed to: %s", ("/"+s+"/odom").c_str()); 
    
    ros::Subscriber sub_map = n.subscribe("/occupancy_map", 5, getMapParams);
    ros::Subscriber nav_sub = n.subscribe("/a_star_goal", 1, nav_receiveNavGoal);

    // Estado inicial de las variables.
    have_map = false;
    nav_move = 0;
    putSensors();
  
    // Escuchamos hasta obtener el mapa, y luego nos desubscribimos.
    while(ros::ok() && !have_map) 
    {
        ros::spinOnce();
        r.sleep();
    }   
    sub_map.shutdown();
  
    // Ciclo normal, sólo escuchando a odom y las metas.
    while (ros::ok())
    {
        ros::spinOnce();
        sensor_marker.publish(drawSensors());
        nav_velocity_pub.publish(navigate());
        vector_marker.publish(drawSpeed(current_vel));
        r.sleep();
    }
}
