/* Mapa básico del salón, usando cubos simples. */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include "Grafica.h"
#include "MapFiller.h"

// La gráfica. 
std::vector<int> GRAPH;
// La gráfica de Voronoi. 
std::vector<Vertice*> VORONOI_GRAPH;

class Celda{
  public:
    int lleno;
    int expandido;
    int x;
    int y;
    int obstacle;
    int seleccionado;
    Vertice* vertice_vor;
    
    Celda() {
      lleno = -1;
      expandido = 0;
      seleccionado = 0;
      vertice_vor = NULL;
    }
};


struct compareCelda
{
  bool operator()(const Celda& lhs, const Celda& rhs) const
  {
    return lhs.lleno > rhs.lleno;
  }
};


void processCell(Celda& p, char* data, int o, int d, 
                 std::priority_queue<Celda, std::vector<Celda>, compareCelda>& cola, 
                 std::vector<int>& vor, bool diag) 
{
  int dif = (diag) ? 3 : 2; 
  if (p.lleno == -1) {
        p.lleno = d;
        p.obstacle = o;
        p.expandido = 1;
        cola.push(p);
        
  } else if(p.lleno >= d-dif) {
      if (p.obstacle != o && data[p.x + p.y*WIDTH] != 100 && p.expandido != 2) {
        vor.push_back(p.x + p.y*WIDTH);
        p.expandido = 2;
        p.seleccionado = 1;
      } 
  }  
}


int addVecino(Celda p, Vertice* v) {
    if (p.seleccionado > 0) {
        int x = p.x;
        int y = p.y;
        Vertice* vec = p.vertice_vor;
        if (vec != NULL) {
          Vecino* vecino = new Vecino(vec);
          (v->vecinos).push_back(vecino);
        }  
        return 1;
    }
    else {
        return 0;
    }
}


std::vector<int> voronoi(char* data, bool manhattan, int* obstacles) 
{

  std::priority_queue<Celda, std::vector<Celda>, compareCelda> cola;
  std::vector<int> vor;
  Celda info[WIDTH][HEIGHT];
  for (int j = 0; j < HEIGHT; j++) {
    for (int i = 0; i < WIDTH; i++) {
      Celda celda;
      info[i][j] = celda;
      info[i][j].x = i;
      info[i][j].y = j;
      info[i][j].obstacle = obstacles[i+j*WIDTH];
      if (data[i + j*WIDTH] == 100) {  
        info[i][j].lleno = 0;
        info[i][j].expandido = 2;
        cola.push(info[i][j]);
      }
    }     
  }
  
  Celda current;
  int i, j, d, previ, prevj, nexti, nextj, i1, j1, i2, j2;
  while(!cola.empty()) {
    current = cola.top();
    cola.pop();
    i = current.x;
    j = current.y;
    int o = info[i][j].obstacle;
    if (info[i][j].expandido < 2 || data[i + j*WIDTH] == 100) {
    
      previ = i-1;
      prevj = j-1;
      nexti = i+1;
      nextj = j+1;
      d = info[i][j].lleno + 2;
      
      if (i != 0)
        processCell(info[previ][j], data, o, d, cola, vor, false);
      
      if (j != 0)
        processCell(info[i][prevj], data, o, d, cola, vor, false);
      
      if (i!= WIDTH - 1)
        processCell(info[nexti][j], data, o, d, cola, vor, false);
      
      if (j != HEIGHT-1)
        processCell(info[i][nextj], data, o, d, cola, vor, false);
              
      if (!manhattan) {
        d = info[i][j].lleno + 3;
        
        if (i != 0 && j != 0)
          processCell(info[previ][prevj], data, o, d, cola, vor, true);
        
        if (i != WIDTH -1 && j != 0)
          processCell(info[nexti][prevj], data, o, d, cola, vor, true);
      
        if (i != WIDTH - 1 && j != HEIGHT -1)
          processCell(info[nexti][nextj], data, o, d, cola, vor, true);
        
      
        if (j != HEIGHT-1 && i != 0)
          processCell(info[previ][nextj], data, o, d, cola, vor, true);
      }
    }
  }
  
  std::vector<int> nodes;
  
  int l = vor.size();
  
  // Creamos todos los vértices de la gráfica. 
  std::vector<Vertice*> grafica_voronoi;
  for (int i = 0; i < l; i++) {
      int x = vor[i] % WIDTH;
      int y = vor[i] / WIDTH;
      Vertice* v = new Vertice(x, y);
      grafica_voronoi.push_back(v);
      info[x][y].vertice_vor = v;
  }
  
  int first = -1;
  
  for (int i = 0; i < l; i++){
    int x = vor[i] % WIDTH;
    int y = vor[i] / WIDTH;
    Vertice* actual = grafica_voronoi[i];
    
    int prevx = x-1;
    int prevy = y-1;
    int nextx = x+1;
    int nexty = y+1;
    int enVor[4];
    
    enVor[0] = addVecino(info[prevx][y], actual);
    enVor[1] = addVecino(info[nextx][y], actual);
    enVor[2] = addVecino(info[x][prevy], actual);
    enVor[3] = addVecino(info[x][nexty], actual);
    
    int total = enVor[0] + enVor[1] + enVor[2] + enVor[3];
    if (total > 2) {
      nodes.push_back(vor[i]);
      (actual->color) = 1;
      if (first == -1)
        first = i;
    }  
  }
  
  
  /**
  for (int i = 0; i<l; i++){
      Vertice* actual = grafica_voronoi[i];
      printf("%d\tx:%d, y:%d, COLOR: %d, VECINOS:", i, actual->x, actual->y, actual->color);
      std::vector<Vecino*> vecinos = (actual->vecinos);
      for (int j = 0; j< (actual->vecinos).size(); j++){
        Vecino* v1 = (actual->vecinos)[j];
        int x1 = (v1->vec)->x;
        int y1 = (v1->vec)->y;          
        printf("x:%d, y:%d; ", x1, y1);
      }
      printf("\n"); 
  }*/
  
  GRAPH = nodes;
  
  VORONOI_GRAPH = prune(grafica_voronoi, first);
  
  l = VORONOI_GRAPH.size();
  
  /**
  for (int i = 0; i < l; i++){
      Vertice* actual = VORONOI_GRAPH[i];
      printf("%d\tx:%d, y:%d, COLOR: %d, VECINOS:", i, actual->x, actual->y, actual->color);
      std::vector<Vecino*> vecinos = (actual->vecinos);
      for (int j = 0; j< (actual->vecinos).size(); j++){
        Vecino* v1 = (actual->vecinos)[j];
        int x1 = (v1->vec)->x;
        int y1 = (v1->vec)->y;          
        printf("x:%d, y:%d; ", x1, y1);
      }
      printf("\n");
  }*/
  
  return vor; 
}

int main( int argc, char** argv )
{
  
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher table_marker_pub = n.advertise<visualization_msgs::MarkerArray>("obstacles_marker", 1);
  ros::Publisher marker_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);

  nav_msgs::OccupancyGrid map;
  visualization_msgs::MarkerArray tables;
  visualization_msgs::Marker front_table;
  
  // La información del mapa.
  map.header.frame_id = "/odom";
  map.header.stamp = ros::Time::now();   // No caduca
  map.info.resolution = RESOLUTION;     // [m/cell]
  map.info.width = WIDTH;               // [cells]
  map.info.height = HEIGHT;             // [cells]
  map.info.origin.position.x = -RESOLUTION*(WIDTH/2.0);
  map.info.origin.position.y = -RESOLUTION*(DOOR_END - 1);
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  
  int size = WIDTH * HEIGHT;
  int* obstacles = new int[size];
  
  for(int i = 0; i < size; i++)
    obstacles[i] = 0;
  
  
  char* data = fillMap(obstacles, size);
  
  std::vector<int> voronoi_squares = voronoi(data, false, obstacles);
  int l = voronoi_squares.size();
  for (int i = 0; i < l; i++){
    int x = voronoi_squares[i] % WIDTH;
    int y = voronoi_squares[i] / WIDTH;
    fillOneRectangle(data, x, y, 150, NULL, 0, false);
  }
  
  l = GRAPH.size();
  for (int i = 0; i < l; i++){
    int x = GRAPH[i] % WIDTH;
    int y = GRAPH[i] / WIDTH;
    fillOneRectangle(data, x, y, 250, NULL, 0, false);
  }
  
  map.data = std::vector<int8_t>(data, data + size);

  while (ros::ok())
  {
    marker_pub.publish(map);
    ros::spinOnce();
    r.sleep();
  }
}
