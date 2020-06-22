/* Mapa del salón. */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include "Grafica.h"
#include "MapFiller.h"
#include "Voronoi_Utils.h"

// La gráfica de VORONOI (con aristas incluidas como vértices). 
std::vector<int> GRAPH;

/**
 * Función auxiliar que realiza el procesamiento de la celda al momento de 
 * expandirla.
 * Recibe como parámetros: 
 *  - La celda p a procesar. 
 *  - Un arreglo data con el valor data de la celda del grid en el mapa,
 *  - Un entero o con el valor del obstáculo que está intentando expandir la onda. 
 *  - Un entero d, el valor que se le asignaría si la onda expande p.
 *  - Una cola de prioridades a la que la celda entrará si es expandida. 
 *  - El vector de voronoi, donde la celda entrará si se ha detectado que está a la 
 *    misma distancia de dos obstáculos diferentes.
 *  - Un booleano que nos dice si la expansión está siendo en diagonal.
 */
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
        if (p.obstacle != o && data[p.x + p.y*WIDTH] != OBSTACULO && p.expandido != 2) {
            vor.push_back(p.x + p.y*WIDTH);
            p.expandido = 2;
            p.seleccionado = 1;
        } 
    }  
}

/**
 * Función que se encarga de la construcción completa de la gráfica de 
 * voronoi. 
 * Recibe la información de los obstáculos del arreglo data, 
 * El booleano manhattan nos dice si expandimos con vecindad manhattan o de
 * 8 vecinos.
 * El arreglo de obstáculos nos dice a qué obstáculo pertenecen las 
 * celdas llenas con OBSTACULO en data.
 * Regresa un vector con las celdas que están en la gráfica de voronoi
 * (incluyendo aristas). Modifica las variables globales GRAPH, en donde se 
 * guardan las celdas que son los nodos de Voronoi y más importantemente, 
 * la variable VORONOI_GRAPH en donde se guarda en efecto, la gráfica de Voronoi.
 */
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
            if (data[i + j*WIDTH] == OBSTACULO) {  
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
        
        if (info[i][j].expandido < 2 || data[i + j*WIDTH] == OBSTACULO) {
          
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
  
    for (int i = 0; i < l; i++) {
        int x = vor[i] % WIDTH;
        int y = vor[i] / WIDTH;
    
        int prevx = x-1;
        int prevy = y-1;
        int nextx = x+1;
        int nexty = y+1;
        
        int enVor[4];
        enVor[0] = (info[prevx][y].seleccionado > 0)? 1 : 0;
        enVor[1] = (info[nextx][y].seleccionado > 0)? 1 : 0;
        enVor[2] = (info[x][prevy].seleccionado > 0)? 1 : 0;
        enVor[3] = (info[x][nexty].seleccionado > 0)? 1 : 0;
    
        int total = enVor[0] + enVor[1] + enVor[2] + enVor[3];
        if (total > 2)
            nodes.push_back(vor[i]);
    }
  
    GRAPH = nodes;
  
    return vor; 
}


int main( int argc, char** argv )
{
    
    /* Creamos los componentes de ROS. */
    ros::init(argc, argv, "basic_map");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher table_marker_pub = n.advertise<visualization_msgs::MarkerArray>("obstacles_marker", 1);
    ros::Publisher marker_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);
    ros::Publisher vor_pub = n.advertise<nav_msgs::OccupancyGrid>("voronoi_info", 1);
    
    /* Creamos los mensajes que mandaremos. */
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid map_for_vor;
    visualization_msgs::MarkerArray tables;
    
    /* Llenamos la información de los mapas.*/
    mapHeader(map);
    mapHeader(map_for_vor);
    
    int size = WIDTH * HEIGHT;
    int* obstacles = new int[size];
  
    for(int i = 0; i < size; i++)
        obstacles[i] = 0;
    
    /* Llenmos la información de los obstáculos en el mapa original. */
    char* data = fillMap(obstacles, size);
    /* Copiamos esa información para el de la información de Voronoi. 
     * deben ser distintos pues el valor de relleno toma una interpretación
     * distinta en general.
     */
    char* data_for_vor = fillMap(obstacles, size);
    
    /* Llenamos las celdas aristas en el mapa de voronoi y para visualizar.*/
    std::vector<int> voronoi_squares = voronoi(data, false, obstacles);
    int l = voronoi_squares.size();
    for (int i = 0; i < l; i++){
        int x = voronoi_squares[i] % WIDTH;
        int y = voronoi_squares[i] / WIDTH;
        // Llenamos con 150 sólo para visualización.
        fillOneRectangle(data, x, y, 150, NULL, 0, false);
        // Indicamos que es parte de una arista.
        fillOneRectangle(data_for_vor, x, y, ARISTA, NULL, 0, false);
    }
  
    /* Llenamos las celdas que son nodos de la gráfica de Voronoi.*/
    l = GRAPH.size();
    for (int i = 0; i < l; i++){
        int x = GRAPH[i] % WIDTH;
        int y = GRAPH[i] / WIDTH;
        // Llenamos con 250 sólo para la visualización.
        fillOneRectangle(data, x, y, 250, NULL, 0, false);
        // Idicamos que detectamos que debe ser nodo.
        fillOneRectangle(data_for_vor, x, y, NODO, NULL, 0, false);
    }
  
    map.data = std::vector<int8_t>(data, data + size);
    tables = addTables(map);

    while (ros::ok())
    {
        marker_pub.publish(map);
        vor_pub.publish(map_for_vor);
        table_marker_pub.publish(tables);
        ros::spinOnce();
        r.sleep();
    }
}
