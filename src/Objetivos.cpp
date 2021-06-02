#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>
#include <iostream>
#include "LogHeader.h"

/** Variables denotando las posiciones de los obstáculos en el mapa.*/ 
const int WIDTH = 27;           // A lo largo del eje rojo x --- 15/0.3 = 4.5 mts
const int HEIGHT = 30;          // A lo largo del eje verde y -- 18/0.3 = 5.4 mts  
const float RESOLUTION = 0.2;   // metros/cuadro
const float BALL_SIZE = 0.3; 
std::vector<double> grabbed;

/**
 * Función que nos regresa el MarkerArray con la información para publicar
 * todos los objetivos en el mapa. 
 */
visualization_msgs::MarkerArray addTargets() 
{
    visualization_msgs::MarkerArray targets;

    for (int id = 0; id < 4; id++){
      visualization_msgs::Marker target;
      target.ns = "Ball_" + std::to_string(id);
      target.scale.x = RESOLUTION*BALL_SIZE;
      target.scale.y = RESOLUTION*BALL_SIZE;
      target.scale.z = RESOLUTION*BALL_SIZE;
      target.pose.position.z = (RESOLUTION*BALL_SIZE)/2;
      targets.markers.push_back(target);
    }
    
    /* Setting common variables for the targets.*/
    int number_targets = targets.markers.size();
    for(int i = 0; i<number_targets; i++){
        targets.markers[i].header.frame_id = "/odom";
        targets.markers[i].id = i;
        targets.markers[i].header.stamp = ros::Time();
        targets.markers[i].type = visualization_msgs::Marker::SPHERE;
        targets.markers[i].action = visualization_msgs::Marker::ADD;    
        targets.markers[i].pose.orientation.x = 0;
        targets.markers[i].pose.orientation.y = 0;
        targets.markers[i].pose.orientation.z = 0;
        targets.markers[i].pose.orientation.w = 1.0;
        targets.markers[i].color.a = 0.8;
        targets.markers[i].color.r = 0.0;
        targets.markers[i].color.g = 0.8;
        targets.markers[i].color.b = 0.0;
      //   targets.markers[i].pose.position.z = RESOLUTION * 0.5;
    }
   targets.markers[0].pose.position.x = -11 * RESOLUTION;
   targets.markers[0].pose.position.y = 1 * RESOLUTION;
   // //targets.markers[0].target.pose.position.z = 3 * (id-5) * RESOLUTION;

   targets.markers[1].pose.position.x = 8 * RESOLUTION;
   targets.markers[1].pose.position.y = 0 * RESOLUTION;
   // //targets.markers[1].target.pose.position.z = 3 * (id-5) * RESOLUTION;

   targets.markers[2].pose.position.x = -3 * RESOLUTION;
   targets.markers[2].pose.position.y = -22 * RESOLUTION;
   // //targets.markers[2].target.pose.position.z = 3 * (id-5) * RESOLUTION;

   targets.markers[3].pose.position.x = 6 * RESOLUTION;
   targets.markers[3].pose.position.y = -12 * RESOLUTION;
   // // targets.markers[3].target.pose.position.z = 3 * (id-5) * RESOLUTION;
    return targets;
}

void grabbed_callback(std_msgs::Float64MultiArray msg){
  grabbed = std::vector<double>(msg.data.begin(),msg.data.end());
}

int main( int argc, char** argv )
{

  ros::init(argc, argv, "targets");
  ros::NodeHandle n;
  ros::Rate r(1);
  // Nos suscribimos para saber si alguna pelota esta agarrada por el robot 
  ros::Subscriber sub_grabbed = n.subscribe("/grabbed_ball", 5, grabbed_callback);
  // Publicamos las pelotas en rviz
  ros::Publisher targets_marker_pub = n.advertise<visualization_msgs::MarkerArray>("targets_marker", 1);

  visualization_msgs::MarkerArray targets;
  
  // Arreglo de marcadores con las pelotas objetivo
  targets = addTargets();

  // Publicamos los markadores y si alguna pelota es agarrada por el robot
  // la hacemos invisible hasta que sepamos donde la dejó el robot
  while (ros::ok())
  {
    // si grabbed es un arreglo con un solo numero significa que 
    // el robot agarró la pelota con dicho número 
    if (grabbed.size() == 1){
      targets.markers[grabbed[0]].color.a = 0.0;

    }
    // si grabbed es un arreglo con más de un numero significa que 
    // el robot dejó la pelota an alguna posición del mapa
    else if (grabbed.size() > 1){
      targets.markers[grabbed[0]].color.a = 0.8;
      targets.markers[grabbed[0]].pose.position.x = grabbed[1];
      targets.markers[grabbed[0]].pose.position.y = grabbed[2];
    }
    targets_marker_pub.publish(targets);
    ros::spinOnce();
    r.sleep();
  }
}