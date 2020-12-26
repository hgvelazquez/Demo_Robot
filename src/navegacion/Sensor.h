#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define _USE_MATH_DEFINES


/**
 * Clase que modela la simulación de los sensores que
 * ocuparemos para detectar colisiones. 
 */
class Sensor {
  
  public:
    int id;
    geometry_msgs::Point current_position;
    geometry_msgs::Point collision_point;
    float distance_detected;
    // Se considera colición a un objeto a una distancia menor a un metro
    bool collision;                          
    tf2::Quaternion angle_vector;
    tf2::Quaternion absolute_angle;
    float angle;
    // Indica si el sensor es de los sensores frontales [45°,-45°]
    bool front;
    
    /**
     * Calcula la distancia euclidiana entre dos puntos en R2.
     * @param p1 Punto inicial desde el cual se quiere medir la distancia.
     * @param p2 Punto final al cual se quiere medir la distancia.
     * @return Distancia del punto p1 al punto p2.
     */
    float distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        float sumx = (p1.x - p2.x)*(p1.x - p2.x);
        float sumy = (p1.y - p2.y)*(p1.y - p2.y); 
        return sqrt(sumx + sumy);
    }
    
    /**
     * Calcula el punto de colisión, actualiza la distancia
     * detectada y marca si estamos a menos de un metro o no. 
     * @param map 
     */
    geometry_msgs::Point getEndPoint(nav_msgs::OccupancyGrid map)
    {
        collision_point = findCollision(map);
        distance_detected = distance(current_position, collision_point);
        collision = (distance_detected < 1);
        return collision_point;
    }
    
    /**
     * Función que se encarga de encontrar el obstáculo con el que colisiona
     * la señal (simulada del sensor). El método consiste en extender una
     * línea sobre la cuadrícula del mapa hasta llegar a algún obstáculo. 
     */
    geometry_msgs::Point findCollision(nav_msgs::OccupancyGrid map)
    {
        geometry_msgs::Point crash;
        
        // Obtenemos variables del mapa para ocuparlas con fácil escritura. 
        int width = map.info.width;
        int height = map.info.height;
        int size = width*size;
        float x = current_position.x - map.info.origin.position.x;
        float y = current_position.y - map.info.origin.position.y;
        float resolution = map.info.resolution;
        // Obtenemos el punto actual de la cuadrícula. 
        int i = (int)(x/resolution);
        int j = (int)(y/resolution);
      
        // atan2 manages all cases where y == 0 or x == 0...
        float angle = atan2(absolute_angle.y(), absolute_angle.x());
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
                    if (i >= width || map.data[j*width + i] == 100) {
                        crash.x = xn + map.info.origin.position.x;
                        crash.y = yn + map.info.origin.position.y;
                        return crash;
                    }
                } else if (yn == j*resolution) {
                    i++;
                    j += (m != 0)? 1 : 0;
                    if (j>=height || i>=width || map.data[j*width+i] == 100){
                        crash.x = xn + map.info.origin.position.x;
                        crash.y = yn + map.info.origin.position.y;
                        return crash;
                    }
                } else {
                    j += (m != 0)? 1 : 0;
                    yn = j*resolution;
                    xn = (m!=0)? (yn - y)/m + x: xn+resolution;
                    if (j >= height || map.data[j*width + i] == 100) {
                        crash.x = xn + map.info.origin.position.x;
                        crash.y = yn + map.info.origin.position.y;
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
                  if (i < 0 || map.data[j*width + i] == 100) {
                      crash.x = xn + map.info.origin.position.x;
                      crash.y = yn + map.info.origin.position.y;
                      return crash;
                  }
              } else if (yn == j*resolution) {
                  i--;
                  j += (m != 0)? 1 : 0;
                  if (j>=height || i<0 || map.data[j*width+i] == 100){
                      crash.x = xn + map.info.origin.position.x;
                      crash.y = yn + map.info.origin.position.y;
                      return crash;
                  }
              } else {
                  j += (m != 0)? 1 : 0;
                  yn = j*resolution;
                  xn = (m!=0)? (yn - y)/m + x: xn-resolution;
                  if (j >= height || map.data[j*width + i] == 100) {
                      crash.x = xn + map.info.origin.position.x;
                      crash.y = yn + map.info.origin.position.y;
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
                      if (i < 0 || map.data[j*width + i] == 100) {
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
                          return crash;
                      }
                  } else if (yn == j*resolution) {
                      i--;
                      j -= (m != -M_PI)? 1 : 0;
                      if (j < 0 || i< 0 || map.data[j*width+i] == 100){
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
                          return crash;
                      }
                  } else {
                      yn = j*resolution;
                      xn = (m != 0)? (yn - y)/m + x: xn-resolution;
                      xn = (xn > -20)? xn : -map.info.origin.position.x;
                      j -= (m != 0)? 1 : 0;
                      if (j < 0 || map.data[j*width + i] == 100) {
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
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
                      if (i >= width || map.data[j*width + i] == 100) {
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
                          return crash;
                        }
                  } else if (yn == j*resolution) {
                      i++;
                      j -= (m != 0)? 1 : 0;
                      if (j < 0 || i >= width || map.data[j*width+i] == 100){
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
                          return crash;
                      }
                  } else {
                      yn = (j)*resolution;
                      j -= (m != 0)? 1 : 0;
                      xn = (m!=0)? (yn - y)/m + x: xn + resolution;
                      if (j < 0 || map.data[j*width + i] == 100) {
                          crash.x = xn + map.info.origin.position.x;
                          crash.y = yn + map.info.origin.position.y;
                          return crash;
                      }
                  }
              }
            }
        }
    }
};

