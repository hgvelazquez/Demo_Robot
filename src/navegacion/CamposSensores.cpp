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

#define _USE_MATH_DEFINES

geometry_msgs::Pose KOBUKI_POSITION;
nav_msgs::OccupancyGrid MAP;
int nav_move;
geometry_msgs::Twist nav_vel;
geometry_msgs::Point nav_goal;
int have_map;
float XI = 0.8;
float ETA = 1.2;
bool turning; // Agregamos esta bandera para que gire hasta que no se oriente al goal (aprox).
// Hay que agregar la parte paera

class Sensor {
  public:
    int id;
    geometry_msgs::Point current_position;
    geometry_msgs::Point collision_point;
    float distance_detected;
    bool collision;
    tf2::Quaternion angle_vector;
    tf2::Quaternion absolute_angle;
    float angle;
    bool front;
    
    float distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
      float sumx = (p1.x - p2.x)*(p1.x - p2.x);
      float sumy = (p1.y - p2.y)*(p1.y - p2.y); 
      return sqrt(sumx + sumy);
    }
    
    geometry_msgs::Point getEndPoint()
    {
      collision_point = findCollision();
      distance_detected = distance(current_position, collision_point);
      collision = (distance_detected < 1);
      return collision_point;
    }
    
    geometry_msgs::Point findCollision()
    {
      geometry_msgs::Point crash;
      
      int width = MAP.info.width;
      int height = MAP.info.height;
      int size = width*size;
      float x = current_position.x - MAP.info.origin.position.x;
      float y = current_position.y - MAP.info.origin.position.y;
      float resolution = MAP.info.resolution;
      int i = (int)(x/0.2);
      int j = (int)(y/0.2);
      
      
      float angle = atan2(absolute_angle.y(), absolute_angle.x());
      // atan2 manages all cases where y == 0 or x == 0...
      float m = tan(angle);
      float xn, yn;
      // We handle the first two quadrants first
      if (angle >= 0) {
        // First Quadrant.
        
        if (angle < M_PI/2){
          // We will move in both positive directions
          while(i < width && j < height) {
            xn = (i+1)*resolution;
            yn = y + m*(xn - x);
            
            if (yn < (j+1)*resolution && yn > j*resolution){
              i++;
              if (i >= width || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else if (yn == j*resolution) {
              i++;
              j += (m != 0)? 1 : 0;
              if (j>=height || i>=width || MAP.data[j*width+i] == 100){
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else {
              j += (m != 0)? 1 : 0;
              yn = j*resolution;
              xn = (m!=0)? (yn - y)/m + x: xn+resolution;
              if (j >= height || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            }
          }
        }
        //Second Quadrant.
        else {
          while(i >= 0 && j < height) {
            xn = i*resolution;
            yn = y + m*(xn - x);
            
            if (yn < (j+1)*resolution && yn >= j*resolution){
              i--;
              if (i < 0 || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else if (yn == j*resolution) {
              i--;
              j += (m != 0)? 1 : 0;
              if (j>=height || i<0 || MAP.data[j*width+i] == 100){
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else {
              j += (m != 0)? 1 : 0;
              yn = j*resolution;
              xn = (m!=0)? (yn - y)/m + x: xn-resolution;
              if (j >= height || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            }
          }
        }
      }
      // We handle quadrants three and fourm angle < 0
      else {
        
        // THIRD Quadrant.
        if (angle <= -M_PI/2){
          // We will move in both positive directions
          while(i>=0 && j >= 0) {
            xn = i*resolution;
            yn = y + m*(xn - x);
            
            if (yn < (j+1)*resolution && yn >= j*resolution){
              i--;
              if (i < 0 || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else if (yn == j*resolution) {
              i--;
              j -= (m != -M_PI)? 1 : 0;
              if (j < 0 || i< 0 || MAP.data[j*width+i] == 100){
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else {
              yn = j*resolution;
              xn = (m != 0)? (yn - y)/m + x: xn-resolution;
              xn = (xn > -20)? xn : -MAP.info.origin.position.x;
              j -= (m != 0)? 1 : 0;
              if (j < 0 || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            }
          }
        }
        //Fourth Quadrant.
        
        else {
          while(i < width && j > 0) {
            xn = (i+1)*resolution;
            yn = y + m*(xn - x);
            
            if (yn < (j+1)*resolution && yn >= j*resolution){
              i++;
              if (i >= width || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else if (yn == j*resolution) {
              i++;
              j -= (m != 0)? 1 : 0;
              if (j < 0 || i >= width || MAP.data[j*width+i] == 100){
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            } else {
              yn = (j)*resolution;
              j -= (m != 0)? 1 : 0;
              xn = (m!=0)? (yn - y)/m + x: xn + resolution;
              if (j < 0 || MAP.data[j*width + i] == 100) {
                crash.x = xn + MAP.info.origin.position.x;
                crash.y = yn + MAP.info.origin.position.y;
                return crash;
              }
            }
          }
        }
      
      }
    }
};

const int SENSOR_NUMBER = 18;
Sensor SENSORS[SENSOR_NUMBER];

void putSensors() {
  float fraction = 2*M_PI/SENSOR_NUMBER;
  tf2::Quaternion rotation;
  rotation.setRPY(0, 0, fraction);
  rotation.normalize();
  tf2::Quaternion v(1, 0, 0, 0);
  geometry_msgs::Pose KO;
  KO.orientation.w = 1.0;
  KOBUKI_POSITION = KO;
  for (int i = 0; i<SENSOR_NUMBER; i++) { 
    Sensor sensor;
    sensor.id = i;
    sensor.angle_vector = v;
    sensor.angle = i*fraction;
    sensor.front = (i*fraction < M_PI/4 || i*fraction > 7*M_PI/4);
    v = rotation*v*inverse(rotation);
    v.normalize();
    tf2::Quaternion v2 = rotation*v*inverse(rotation);
    SENSORS[i] = sensor;
  }  
} 

void getMapParams(const  nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  MAP = *msg; 
  have_map = 1;
}

void updateSensors(const nav_msgs::Odometry::ConstPtr& msg) 
{
  geometry_msgs::Pose pose;
  nav_msgs::Odometry od = *msg;
  pose = od.pose.pose;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  q.normalize();
  for (int i = 0; i < SENSOR_NUMBER; i++) 
  {
    tf2::Quaternion new_angle = q*SENSORS[i].angle_vector*inverse(q);
    new_angle.normalize();
    SENSORS[i].absolute_angle = new_angle;
    SENSORS[i].current_position = pose.position;
  }
  KOBUKI_POSITION = pose;
  if(SENSORS[0].distance(pose.position, nav_goal) < 0.3) {
    nav_move = 0;
    nav_vel.linear.x = 0;
    nav_vel.angular.z = 0;
  }  
}

visualization_msgs::MarkerArray drawSensors()
{
  visualization_msgs::MarkerArray sensors;
  geometry_msgs::Twist nav_new;
  geometry_msgs::Pose point = KOBUKI_POSITION;
  tf2::Quaternion v1(point.orientation.x, point.orientation.y, point.orientation.z, point.orientation.w);
  geometry_msgs::Point ZERO;
  int count = 0;
  float min = 100;
  int m = SENSOR_NUMBER;
  
  for(int i = 0; i < SENSOR_NUMBER; i++) {
    
    visualization_msgs::Marker current_sensor;
    //General data of the Line
    
    current_sensor.header.frame_id = "/base_link";
    current_sensor.header.stamp = ros::Time::now();   // No caduca
    current_sensor.ns = "sensor";  
    current_sensor.type = visualization_msgs::Marker::LINE_LIST;
    current_sensor.action = visualization_msgs::Marker::ADD;
    current_sensor.pose.orientation.w = 1; // All the others are zero.
    //current_sensor.pose.orientation = KOBUKI_POSITION.orientation;
    
    current_sensor.scale.x = 0.02;
    current_sensor.color.a = 1.0; // Same transparency for all. 
    
    // We put the start point and end point.
    
    current_sensor.points.push_back(ZERO);
    
    SENSORS[i].getEndPoint();
    
    geometry_msgs::Point end;
    
    float angle = SENSORS[i].angle;
    end.x = SENSORS[i].distance_detected*cos(angle); 
    end.y = SENSORS[i].distance_detected*sin(angle);

    current_sensor.points.push_back(end);
    
    geometry_msgs::Point actual;// 0 al inicializarse.
    
    float nav_d0 = 0.9;
    if (nav_move == 1) {
      //actual.x = 0.2*cos(SENSORS[i].angle); 
      //actual.y = 0.2*sin(SENSORS[i].angle);
      
      float nav_d = SENSORS[i].distance_detected - 0.2; // Movemos los sensores a la orilla de la KOBUKI.
      if (nav_d < min) {
        min =  nav_d;
        m = i;
      }
      
      float weight = 2*fabs(M_PI - angle)/6;
      printf("%2.3f %2.3f\n",angle, weight);
      if (nav_d < nav_d0) {
        count++;
        nav_d = (nav_d < 0.00001) ? 0.00001 : nav_d; // Tope a la fuerza repulsora.
          
        if (nav_d < 0.4 && SENSORS[i].front) 
          nav_d *= 0.01;
        else if (nav_d < 0.4) 
          nav_d *= 0.1; 
        
        float factor = ETA*((1/nav_d) - (1/nav_d0))*(1/nav_d*nav_d*nav_d);  
        
        if (SENSORS[i].front)
          nav_new.linear.x += weight*factor*(-end.x);
        nav_new.angular.z += weight*factor*(-end.y);
        }  
    }
    
    current_sensor.id = i;
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
  
  // Pasamos el goal al marco de la KOBUKI.
  geometry_msgs::Point nav_goal_coord;
  nav_goal_coord.x = nav_goal.x -  point.position.x;
  nav_goal_coord.y = nav_goal.y -  point.position.y;
  tf2::Quaternion g(nav_goal_coord.x, nav_goal_coord.y, 0, 0);
  g = inverse(v1)*g*v1;
  nav_goal_coord.x = g.x();
  nav_goal_coord.y = g.y();
  
  if (nav_move == 1) {
    float nav_d = SENSORS[0].distance(ZERO, nav_goal_coord);
    float deltax = -nav_goal_coord.x;
    float deltay = - nav_goal_coord.y;
    if (nav_d > 2) {
        nav_new.linear.x -= 0.5*XI*deltax;
        nav_new.angular.z -= 0.5*XI*deltay;
      }
      else if (nav_d < 0.4) {
        nav_new.linear.x -= 6*XI*deltax;
        nav_new.angular.z -= 6*XI*deltay;
      }else {  
        nav_new.linear.x -= 2*XI*deltax;
        nav_new.angular.z -= 2*XI*deltay;
      }
      
      float sumx = 0.7*nav_vel.linear.x + 0.3*nav_new.linear.x;
      float sumz = 0.7*nav_vel.angular.z + 0.3*nav_new.angular.z;
      float norm = sumx*sumx + sumz*sumz;
      norm = sqrt(norm);
      tf2::Vector3 v1(sumx, sumz, 0);
      tf2::Vector3 v2(nav_vel.linear.x, nav_vel.angular.z, 0);
      /**
      if (sumx < 0) {
          nav_vel.linear.x = 0;
          nav_vel.angular.z = M_PI      
      }*/
      if (v2.angle(v1) > M_PI/4) {
          nav_vel.linear.x = 0;//sumx/20*norm;
          nav_vel.angular.z = 19*sumz/20*norm;
      } else {
        nav_vel.linear.x = sumx/(norm);
        nav_vel.angular.z = sumz/(norm);
      }
  }
  
  else {
    nav_vel.linear.x = 0;
    nav_vel.angular.z = 0;
  }
  
    visualization_msgs::Marker nav_sensor;
    //General data of the Line
    nav_sensor.header.frame_id = "/base_link";
    nav_sensor.header.stamp = ros::Time::now();   // No caduca
    nav_sensor.ns = "vel";  
    nav_sensor.type = visualization_msgs::Marker::ARROW;
    nav_sensor.action = visualization_msgs::Marker::ADD;
    nav_sensor.pose.orientation.w = 1; // All the others are zero.
    //current_sensor.pose.orientation = KOBUKI_POSITION.orientation;
    
    nav_sensor.scale.x = 0.05;
    nav_sensor.scale.y = 0.1;
    nav_sensor.scale.z = 0.1;
    nav_sensor.color.a = 1.0; // Same transparency for all. 
    nav_sensor.color.b = 1.0;
    // We put the start point and end point.
    
    nav_sensor.points.push_back(ZERO);
    
    geometry_msgs::Point end;
    
    end.x = nav_vel.linear.x; 
    end.y = nav_vel.angular.z;
    
    nav_sensor.points.push_back(end);
    nav_sensor.id = 20;
    sensors.markers.push_back(nav_sensor);
  
  
  
  return sensors;
}  

void nav_receiveNavGoal(const geometry_msgs::Point& goalStamped)
{
  nav_goal = goalStamped;      
  nav_move = 1;
}

int main (int argc, char** argv)
{
  
  ros::init(argc, argv, "collisions");
  ros::NodeHandle n;
  ros::Rate r(20);
  
  ros::Publisher sensor_marker = n.advertise<visualization_msgs::MarkerArray>("sensor_markers", 10);
  ros::Subscriber sub_odom = n.subscribe("/odom", 5, updateSensors);
  ros::Subscriber sub_map = n.subscribe("/occupancy_map", 5, getMapParams);
  ros::Publisher nav_velocity_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Subscriber nav_sub = n.subscribe("/a_star_goal", 1, nav_receiveNavGoal);

  have_map = 0;
  nav_move = 0;
  putSensors();
  
  while (ros::ok())
  {
    ros::spinOnce();
    if (have_map != 0)
      sensor_marker.publish(drawSensors());
    nav_velocity_pub.publish(nav_vel);
    r.sleep();
  }
}
