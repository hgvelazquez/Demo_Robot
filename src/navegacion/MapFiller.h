/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

/* RViz usa un sistema de coordenadas usual, no el de graficación.
(0,
HEIGHT) -------------------------------------------- (HEIGHT, WIDTH)
        |             FRONT                |       | 
        |             TABLE                |       | SERVER ROOMS
        |                                  | "L"   | TO THIS SIDE
DOOR_   D----.-----------------------------| TABLE | (UNACCESIBLE)
END     O    .                             |       |
DOOR_   O    .                             |-------|
START   R----.        ----------------             |
        | S  |        |              |             |
        | I  |        |              |             |
        | D  |        |   CENTER     |             |
        | E  |        |   TABLE      |             |
        |    |        |              |             |
        | T  |        |              |             |
        | A  |        |              |             |
        | B  |        |              |             |
        | L  |        |              |             |
        | E  |        |              |    |--------|
        |    |        |              |    |        |
        |    |        ---------------     |  BACK  |
        |    |                            |  TABLE |
        |    |                            |        |
        |    |                            |        |
(0,0)   --------------------BOARD------------------- (WIDTH, 0)
*/

#include <tf2/LinearMath/Quaternion.h>


const float RESOLUTION = 0.2;   // metros/cuadro
/** Variables denotando las posiciones de los obstáculos en el mapa.*/ 
// se le suma a la altura y al ancho 2 cuadros al inicio 
// y al final para representar las paredes en lo bordes
// del mapa
// Y se multiplica por el inverso multiplicativo 
// de la resolución pues un número de cuadros 
// inversamente proporcional a la resolución para representar
// un cuadro de 1 metro.
const int WIDTH = (14+2)*(1/RESOLUTION);           // A lo largo del eje rojo x --- 15/0.3 = 4.5 mts
const int HEIGHT = (8+2)*(1/RESOLUTION);//30;//          // A lo largo del eje verde y -- 18/0.3 = 5.4 mts 
//const int WIDTH = 27;
//const int HEIGHT = 30;


// Coordenadas del tamaño de la puerta.
const int DOOR_START = (HEIGHT-1) - 8;
const int DOOR_END = (HEIGHT-1) - 4;

// Coordenadas de la mesa central... ¿Usar patas o mesa completa?
const int CENTRAL_X_START = 9;
const int CENTRAL_Y_START = 5;
const int CENTRAL_X_END = WIDTH-9;
const int CENTRAL_Y_END = (HEIGHT-1)-8;

// Coordenadas para la mesa al ladp de la puerta
const int SIDE_TABLE_END = 3;

// Coordenadas para la mesa de enfrente. Está contra dos paredes, necesitamos dos coordenadas.
const int FRONT_TABLE_X_END = 19;
const int FRONT_TABLE_Y_START = (HEIGHT-1) - 3;

// La mesa en posición de L respecto a la mesa de enfrente.
const int L_TABLE_X_START = FRONT_TABLE_X_END+4;
const int L_TABLE_Y_START = HEIGHT-10;

// La mesa en la parte de atrás.
const int BACK_TABLE_Y_END = 8;
const int BACK_TABLE_X_START = FRONT_TABLE_X_END+4;

// El número de obstaculo (información necesaria para ir añadiendo los obstáculos)
int obstacle_count;


/** 
 * Función encargada de llenar el rectángulo 
 * (agregar el obstaculo) que va de i1 a i2 y de j1 a j2, de 
 * manera inclusiva, con el valor value.
 * @param data arreglo con los valores necesarios para visualizar 
 * cada cuadro del mundo.
 * @param i1 cordenada x desde la que se empieza a "rellenar".
 * @param j1 cordenada y desde la que se empieza a "rellenar".
 * @param i2 cordenada x donde se termina de "rellenar".
 * @param j2 cordenada y donde se termina de "rellenar".
 * @param value valor que se le dará en el arreglo data a los cuadros en la zona 
 * abarcada.
 * @param obstacles Arreglo cuyos valores representan el id del obstaculo 
 * que se va a agregar. 
 * @param obstacle Identificador del obstaculo que se está agregando.
 * @param fill Si es verdadero, y value es 100, marca el 
 * rectángulo como un obstáculo. 
 */
void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value, int* obstacles, int obstacle, bool fill)
{
  for(int i = i1; i <= i2; i++)
  {
    for(int j = j1; j <= j2; j++)
    {
      if (data[j*WIDTH+i] != 100)
        data[j*WIDTH+i] = value;
      if (value == 100 && fill)
        obstacles[j*WIDTH + i] = obstacle;
    }
  }
}

/**
 * Esta función tiene la misma semántica que fillRectangle, pero sólo llena un 
 * único cuadrado de la celda. 
 * Esto facilita la escritura para cuando tenemos obstáculos muy pequeños.
 * @param data arreglo con los valores necesarios para visualizar 
 * cada cuadro del mundo.
 * @param i cordenada x del cuadro que se quiere rellenar.
 * @param j cordenada y del cuadro que se quiere rellenar.
 * @param value valor que se le dará en el arreglo data a los cuadros en la zona 
 * abarcada.
 * @param obstacles Arreglo cuyos valores representan el id del obstaculo 
 * que se va a agregar. 
 * @param obstacle Identificador del obstaculo que se está agregando.
 * @param fill Si es verdadero, y value es 100, marca el 
 * rectángulo como un obstáculo.
 */
void fillOneRectangle(char* data, int i, int j, int value, int* obstacles, int obstacle, bool fill)
{
  data[j*WIDTH+i] = value;
  // if (data[j*WIDTH+i] != 100)
  //       data[j*WIDTH+i] = value;
  if (value == 100 && fill)
    obstacles[j*WIDTH + i] = obstacle;
}

/**
 * Función que asigna las dimensiones 
 * y resolución al objeto map.
 * @param map Objeto de tipo OccupancyGrid
 * el cual contiene la información del mundo.
 * @param resolution valor de la resolución
 * del mapa.
 * @param width Tamaño del ancho del mapa
 * en número de cuadros de tamaño resolution.
 * @param height Tamaño de la altura del mapa
 * en número de cuadros de tamaño resolution.
 */
void mapDimensions(nav_msgs::OccupancyGrid& map, float resolution, int width, int height){
  map.info.resolution = resolution;     // [m/cell]
  map.info.width = width;               // [cells]
  map.info.height = height;             // [cells]
}

/**
 * Función que asigna el punto de origen 
 * al objeto map.
 * @param map Objeto de tipo OccupancyGrid
 * el cual contiene la información del mundo.
 * @param x valor en el eje x del punto 
 * de origen a asignar.
 * @param y valor en el eje y del punto 
 * de origen a asignar.
 * @param z valor en el eje z del punto 
 * de origen a asignar.
 */
void mapOrigin(nav_msgs::OccupancyGrid& map, float x, float y, float z){
  double resolution = map.info.resolution;
  map.info.origin.position.x = -resolution*x;
  map.info.origin.position.y = -resolution*y;
  map.info.origin.position.z = -resolution*z;
}

/**
 * Función que asigna los valores de 
 * orientación del origen 
 * al objeto map.
 * @param map Objeto de tipo OccupancyGrid
 * el cual contiene la información del mundo.
 * @param x valor en el eje x de la orientación 
 * del origen a asignar.
 * @param y valor en el eje y de la orientación 
 * del origen a asignar.
 * @param z valor en el eje de la orientación 
 * del origen a asignar.
 * @param w valor en el eje de la orientación 
 * del origen a asignar.
 */
void mapOrientation(nav_msgs::OccupancyGrid& map, float x, float y, float z, float w){
  map.info.origin.orientation.x = x;
  map.info.origin.orientation.y = y;
  map.info.origin.orientation.z = z;
  map.info.origin.orientation.w = w;
}

void mapRPYOrientation(nav_msgs::OccupancyGrid& map, float x, float y, float z){
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( x, y, z ); 
  myQuaternion.normalize(); 
  map.info.origin.orientation.x = myQuaternion[0];
  map.info.origin.orientation.y = myQuaternion[1];
  map.info.origin.orientation.z = myQuaternion[2];
  map.info.origin.orientation.w = myQuaternion[3];
}

/**
 * Función que da valores al objeto map
 * con la información default 
 * del mapa que usaremos con los valores default
 * para WIDTH, HEIGHT y RESOLUTION, además
 * de una posición inicial, que se obtiene a
 * partir de estas variables y DOOR_END, 
 * valores necesarios para el ejemplo del
 * aula de clases (el vector con los valores de 
 * ocupación no se toca).
 * @param map Objeto de tipo OccupancyGrid
 * el cual contiene la información del mundo.
 */
void mapHeader(nav_msgs::OccupancyGrid& map){
    // La información del mapa.
    // map.header.frame_id = "/odom";
    // map.header.stamp = ros::Time::now();   // No caduca
    // map.info.resolution = RESOLUTION;     // [m/cell]
    // map.info.width = WIDTH;               // [cells]
    // map.info.height = HEIGHT;             // [cells]
    // map.info.origin.position.x = -RESOLUTION*(WIDTH/2.0);
    // map.info.origin.position.y = -RESOLUTION*(DOOR_END - 1);
    // map.info.origin.position.z = 0;
    // map.info.origin.orientation.x = 0.0;
    // map.info.origin.orientation.y = 0.0;
    // map.info.origin.orientation.z = 0.0;
    // map.info.origin.orientation.w = 1.0;

    // // La información del mapa.
    map.header.frame_id = "/odom";
    map.header.stamp = ros::Time::now();   // No caduca
    mapDimensions(map, RESOLUTION, WIDTH, HEIGHT);
    mapOrigin(map, (WIDTH/2.0), (DOOR_END - 1), 0);
    mapOrientation(map, 0.0, 0.0, 0.0, 1.0);
}


/**
 * Función encargada de "colocar" las paredes del mapa.
 * @param obstacles Arreglo de tipo int de tamaño size,
 * cuyo valor representa el id del obstaculo
 * al que pertenece.
 * @param size Tamaño del area del mapa (un número entero
 * para que cada unidad represente un "cuadro" de la cuadricula
 * que representa al mapa).
 * @returns data arreglo con los valores necesarios para visualizar 
 * cada cuadro del mundo.
 */ 
void fillMapWalls(char* data, int* obstacles) 
{
  /* Agregamos la información de las paredes dandole un id de 
  obstaculo a cada una
  */
  fillRectangle(data, 0, 0, WIDTH-1, 0, 100, obstacles, obstacle_count++, true);
  fillRectangle(data, 0, 0, 0, HEIGHT-1, 100, obstacles, obstacle_count++, true);
  fillRectangle(data, 0, HEIGHT-1, WIDTH-1, HEIGHT-1, 100, obstacles, obstacle_count++, true);
  fillRectangle(data, WIDTH-1, 0, WIDTH-1, HEIGHT-1, 100, obstacles, obstacle_count++, true);
  //fillRectangle(data, 0, DOOR_END+1, 0, HEIGHT-1, 100, obstacles, obstacle_count++, true);
}

/**
 * Esta función tiene la misma semántica que fillRectangle, pero sólo llena un 
 * único cuadrado de la celda. 
 * Esto facilita la escritura para cuando tenemos obstáculos muy pequeños.
 * @param data arreglo con los valores necesarios para visualizar 
 * cada cuadro del mundo.
 * @param i cordenada x del cuadro que se quiere rellenar.
 * @param j cordenada y del cuadro que se quiere rellenar.
 * @param obstacles Arreglo cuyos valores representan el id del obstaculo 
 * que se va a agregar. 
 */
void addMachine(char* data, int i, int j, int* obstacles)
{
  fillRectangle(data, i, j, (i+1), (j+1), 100, obstacles, obstacle_count++, true);
  fillRectangle(data, i, (WIDTH-j), (i+1), (WIDTH-j)-1, 100, obstacles, obstacle_count++, true);
}

/**
 * Función encargada de llenar la información del mapa, así como 
 * de poner los obstáculos en el grid correspondiente a cada una
 * de las mesas.
 * @param obstacles Arreglo de tipo int de tamaño size,
 * cuyo valor representa el id del obstaculo
 * al que pertenece.
 * @param size Tamaño del area del mapa (un número entero
 * para que cada unidad represente un "cuadro" de la cuadricula
 * que representa al mapa).
 * @returns 
 */ 
char* fillMap(int* obstacles, int size) 
{  
    obstacle_count = 0;
  
    char* data = new char[size];

    for(int i = 0; i < size; i++) {
        data[i] = 0;
    }
  
    // Marcamos las paredes del salón
    fillRectangle(data, 0, 0, WIDTH-1, 0, 100, obstacles, obstacle_count++, true);
    fillRectangle(data, 0, 0, 0, DOOR_START-1, 100, obstacles, obstacle_count++, true);
    fillRectangle(data, 0, HEIGHT-1, WIDTH-1, HEIGHT-1, 100, obstacles, obstacle_count++, true);
    fillRectangle(data, WIDTH-1, 0, WIDTH-1, HEIGHT-1, 100, obstacles, obstacle_count++, true);
    fillRectangle(data, 0, DOOR_END+1, 0, HEIGHT-1, 100, obstacles, obstacle_count++, true);
    
  
    // Llenamos las patas de la mesa de enfrente
    fillOneRectangle(data, 1, FRONT_TABLE_Y_START, 100, obstacles, 4, true);
    fillOneRectangle(data, 1, HEIGHT-2, 100, obstacles, 4, true);
    fillOneRectangle(data, FRONT_TABLE_X_END, FRONT_TABLE_Y_START, 100, obstacles, obstacle_count, true);
    fillOneRectangle(data, FRONT_TABLE_X_END, HEIGHT-2, 100, obstacles, obstacle_count++, true);
    
    //Patas de En Medio..
    //fillOneRectangle(data, (FRONT_TABLE_X_END)/2 + 1, FRONT_TABLE_Y_START, 100, obstacles, obstacle_count, true);
    //fillOneRectangle(data, (FRONT_TABLE_X_END)/2 + 1, HEIGHT-2, 100, obstacles, obstacle_count++, true);
    
    // Llenamos las patas de la mesa en L
    fillOneRectangle(data, L_TABLE_X_START, L_TABLE_Y_START, 100, obstacles, obstacle_count, true);
    fillOneRectangle(data, L_TABLE_X_START, HEIGHT-2, 100, obstacles, 2, true);
    fillOneRectangle(data, WIDTH-2, L_TABLE_Y_START, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, WIDTH-2, HEIGHT-2, 100, obstacles, 2, true);
    
    // Llenamos las patas de la mesa lateral
    fillOneRectangle(data, 1, 1, 100, obstacles, 0, true);
    fillOneRectangle(data, SIDE_TABLE_END, 1, 100, obstacles, 0, true);
    fillOneRectangle(data, 1, DOOR_START-1, 100, obstacles, obstacle_count, true);
    fillOneRectangle(data, SIDE_TABLE_END, DOOR_START-1, 100, obstacles, obstacle_count++, true);
  
    //Patas de En Medio..
    fillOneRectangle(data, SIDE_TABLE_END, (DOOR_START)/2, 100, obstacles, obstacle_count, true);
    fillOneRectangle(data, 1, (DOOR_START)/2, 100, obstacles, obstacle_count++, true);
  
    // Llenamos las patas de la mesa de atrás
    fillOneRectangle(data, BACK_TABLE_X_START, BACK_TABLE_Y_END, 100, obstacles, obstacle_count, true);
    fillOneRectangle(data, BACK_TABLE_X_START, 1, 100, obstacles, 0, true);
    fillOneRectangle(data, WIDTH-2, BACK_TABLE_Y_END, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, WIDTH-2, 1, 100, obstacles, 0, true);
  
    // Llenamos las patas de la mesa central
    fillOneRectangle(data, CENTRAL_X_START, CENTRAL_Y_START, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, CENTRAL_X_START, CENTRAL_Y_END, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, CENTRAL_X_END, CENTRAL_Y_START, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, CENTRAL_X_END, CENTRAL_Y_END, 100, obstacles, obstacle_count++, true);
    
    // Patas de en Medio
    int halfX = (CENTRAL_X_START+CENTRAL_X_END)/2;
    int halfY = (CENTRAL_Y_START + CENTRAL_Y_END)/2;
    fillRectangle(data, halfX, CENTRAL_Y_START, 1+halfX, CENTRAL_Y_START,100, obstacles, obstacle_count++, true);
    fillRectangle(data, halfX, CENTRAL_Y_END,  1+halfX, CENTRAL_Y_END, 100, obstacles, obstacle_count++, true);
  
    // Centrales
    fillOneRectangle(data, CENTRAL_X_START, halfY, 100, obstacles, obstacle_count++, true);
    fillOneRectangle(data, CENTRAL_X_END, halfY, 100, obstacles, obstacle_count++, true);
    fillRectangle(data, halfX, halfY, 1+ halfX, halfY, 100, obstacles, obstacle_count++, true);
  
    return data;
}



/**
 * Función que nos regresa el MarkerArray con la información para publicar
 * todas las mesas del mapa. 
 */
visualization_msgs::MarkerArray addTables(nav_msgs::OccupancyGrid& map) 
{
    visualization_msgs::MarkerArray tables;
    /* Setting Front Table Position. */  
    visualization_msgs::Marker front_table;
    front_table.ns = "front_table";
    front_table.scale.x = RESOLUTION*(FRONT_TABLE_X_END);
    front_table.scale.y = RESOLUTION*(HEIGHT-FRONT_TABLE_Y_START-1);
    // (0,0 in the map and then put it in position)
    front_table.pose.position.x = map.info.origin.position.x + front_table.scale.x/2 + RESOLUTION;
    front_table.pose.position.y = map.info.origin.position.y + RESOLUTION*FRONT_TABLE_Y_START + front_table.scale.y/2;
  
    tables.markers.push_back(front_table);
  
    /*Setting Center Table Position.*/  
    visualization_msgs::Marker center_table;
    center_table.ns = "center_table";
    center_table.scale.x = RESOLUTION*(CENTRAL_X_END - CENTRAL_X_START + 1);
    center_table.scale.y = RESOLUTION*(CENTRAL_Y_END - CENTRAL_Y_START + 1);
    // (0,0 in the map and then put it in position)
    center_table.pose.position.x = map.info.origin.position.x + center_table.scale.x/2 + RESOLUTION*CENTRAL_X_START;
    center_table.pose.position.y = map.info.origin.position.y + center_table.scale.y/2 + RESOLUTION*CENTRAL_Y_START;
  
    tables.markers.push_back(center_table);
    
    /*Setting Side Table Position. */  
    visualization_msgs::Marker side_table;
    side_table.ns = "side_table";
    side_table.scale.x = RESOLUTION*SIDE_TABLE_END;
    side_table.scale.y = RESOLUTION*(DOOR_START-1);
    // (0,0 in the map and then put it in position)
    side_table.pose.position.x = map.info.origin.position.x + side_table.scale.x/2 + RESOLUTION;
    side_table.pose.position.y = map.info.origin.position.y + side_table.scale.y/2 + RESOLUTION;
    
    tables.markers.push_back(side_table);
  
    /*Setting L Table Position. */
    visualization_msgs::Marker l_table;
    l_table.ns = "l_table";
    l_table.scale.x = RESOLUTION*(WIDTH - L_TABLE_X_START - 1);
    l_table.scale.y = RESOLUTION*(HEIGHT- L_TABLE_Y_START - 1);
    // (0,0 in the map and then put it in position)
    l_table.pose.position.x = map.info.origin.position.x + l_table.scale.x/2 + RESOLUTION*L_TABLE_X_START;
    l_table.pose.position.y = map.info.origin.position.y + l_table.scale.y/2 + RESOLUTION*(L_TABLE_Y_START);
  
    tables.markers.push_back(l_table); 
  
    /* Setting Back Table Position.*/
    visualization_msgs::Marker back_table;
    back_table.ns = "back_table";
    back_table.scale.x = RESOLUTION*(WIDTH - 1 - BACK_TABLE_X_START);
    back_table.scale.y = RESOLUTION*(BACK_TABLE_Y_END);
    // (0,0 in the map and then put it in position)
    back_table.pose.position.x = map.info.origin.position.x + back_table.scale.x/2 + RESOLUTION*BACK_TABLE_X_START;
    back_table.pose.position.y = map.info.origin.position.y + back_table.scale.y/2 + RESOLUTION;
    
    tables.markers.push_back(back_table);
    
    /* Setting common variables for the tables.*/
    int number_tables = tables.markers.size();
    for(int i = 0; i<number_tables; i++){
        tables.markers[i].header.frame_id = "/odom";
        tables.markers[i].id = i;
        tables.markers[i].header.stamp = ros::Time();
        tables.markers[i].type = visualization_msgs::Marker::CUBE;
        tables.markers[i].action = visualization_msgs::Marker::ADD;    
        tables.markers[i].pose.orientation.x = 0;
        tables.markers[i].pose.orientation.y = 0;
        tables.markers[i].pose.orientation.z = 0;
        tables.markers[i].pose.orientation.w = 1.0;
        tables.markers[i].color.a = 0.8;
        tables.markers[i].color.r = 0.8;
        tables.markers[i].color.g = 0.8;
        tables.markers[i].color.b = 0.8;
        tables.markers[i].scale.z = 0.2;
        tables.markers[i].pose.position.z = tables.markers[i].scale.z/2 + 0.5;
    }
  
    /* Now we have to put the legs. 4 auxiliary variables.*/  
    visualization_msgs::Marker table_legs0;
    visualization_msgs::Marker table_legs1;
    visualization_msgs::Marker table_legs2;
    visualization_msgs::Marker table_legs3;
    /* Setting the edge legs position for each table */
    for(int i = 0; i<number_tables; i++){   
        table_legs0.ns = "table_leg_0";
        table_legs1.ns = "table_leg_1";
        table_legs2.ns = "table_leg_2";
        table_legs3.ns = "table_leg_3";
        table_legs0.pose.position.x = tables.markers[i].pose.position.x - tables.markers[i].scale.x/2 + 0.1;
        table_legs0.pose.position.y = tables.markers[i].pose.position.y - tables.markers[i].scale.y/2 + 0.1;
        table_legs1.pose.position.x = tables.markers[i].pose.position.x - tables.markers[i].scale.x/2 + 0.1;
        table_legs1.pose.position.y = tables.markers[i].pose.position.y + tables.markers[i].scale.y/2 - 0.1;
        table_legs2.pose.position.x = tables.markers[i].pose.position.x + tables.markers[i].scale.x/2 - 0.1;
        table_legs2.pose.position.y = tables.markers[i].pose.position.y - tables.markers[i].scale.y/2 + 0.1;
        table_legs3.pose.position.x = tables.markers[i].pose.position.x + tables.markers[i].scale.x/2 - 0.1;
        table_legs3.pose.position.y = tables.markers[i].pose.position.y + tables.markers[i].scale.y/2 - 0.1;
        tables.markers.push_back(table_legs0);
        tables.markers.push_back(table_legs1);
        tables.markers.push_back(table_legs2);
        tables.markers.push_back(table_legs3);
    }
  
  //Patas centrales Mesa de Enfrente y la mesa central (sobre el eje x)
    for (int i = 0; i<2; i++) {  
        float offset = (i==1) ? 0.1 : 0.0; // Sólo hay que mover las de la de en medio.
        table_legs0.pose.position.x = tables.markers[i].pose.position.x - offset;
        table_legs0.pose.position.y = tables.markers[i].pose.position.y - tables.markers[i].scale.y/2 + 0.1;
        table_legs1.pose.position.x = tables.markers[i].pose.position.x - offset;
        table_legs1.pose.position.y = tables.markers[i].pose.position.y + tables.markers[i].scale.y/2 - 0.1;
        tables.markers.push_back(table_legs0);
        tables.markers.push_back(table_legs1);
        if (i == 1) { // Sólo la mesa central (tiene dos)
            table_legs2.pose.position.x = tables.markers[i].pose.position.x + offset;
            table_legs2.pose.position.y = tables.markers[i].pose.position.y - tables.markers[i].scale.y/2 + 0.1;
            table_legs3.pose.position.x = tables.markers[i].pose.position.x + offset;
            table_legs3.pose.position.y = tables.markers[i].pose.position.y + tables.markers[i].scale.y/2 - 0.1;
            tables.markers.push_back(table_legs2);
            tables.markers.push_back(table_legs3);   
        }
    }  


    // Patas Centrales Mesa del Lado 
    table_legs0.pose.position.x = tables.markers[2].pose.position.x - tables.markers[2].scale.x/2 + 0.1;
    table_legs0.pose.position.y = tables.markers[2].pose.position.y - 0.1;
    table_legs1.pose.position.x = tables.markers[2].pose.position.x + tables.markers[2].scale.x/2 - 0.1;
    table_legs1.pose.position.y = tables.markers[2].pose.position.y - 0.1;
    tables.markers.push_back(table_legs0);
    tables.markers.push_back(table_legs1);
  
    // Demás patas centrales de la mesa del centro  
    table_legs0.pose.position.x = tables.markers[1].pose.position.x - tables.markers[1].scale.x/2 + 0.1;
    table_legs0.pose.position.y = tables.markers[1].pose.position.y;
    table_legs1.pose.position.x = tables.markers[1].pose.position.x + 0.1;
    table_legs1.pose.position.y = tables.markers[1].pose.position.y;
    table_legs2.pose.position.x = tables.markers[1].pose.position.x + tables.markers[1].scale.x/2 - 0.1;
    table_legs2.pose.position.y = tables.markers[1].pose.position.y;
    table_legs3.pose.position.x = tables.markers[1].pose.position.x - 0.1;
    table_legs3.pose.position.y = tables.markers[1].pose.position.y;
    tables.markers.push_back(table_legs0);
    tables.markers.push_back(table_legs1);
    tables.markers.push_back(table_legs2);
    tables.markers.push_back(table_legs3);   
 
    /* Setting common variables for each table leg. */ 
    for(int i = number_tables; i<tables.markers.size(); i++) {
        tables.markers[i].header.frame_id = "/odom";
        tables.markers[i].id = i;
        tables.markers[i].header.stamp = ros::Time();
        tables.markers[i].type = visualization_msgs::Marker::CUBE;
        tables.markers[i].action = visualization_msgs::Marker::ADD;    
        tables.markers[i].pose.orientation.x = 0;
        tables.markers[i].pose.orientation.y = 0;
        tables.markers[i].pose.orientation.z = 0;
        tables.markers[i].pose.orientation.w = 1.0;
        tables.markers[i].color.a = 0.8;
        tables.markers[i].color.r = 0.8;
        tables.markers[i].color.g = 0.8;
        tables.markers[i].color.b = 0.8;
        tables.markers[i].scale.x = 0.15;
        tables.markers[i].scale.y = 0.15;
        tables.markers[i].scale.z = 0.6;
        tables.markers[i].pose.position.z = tables.markers[i].scale.z/2;
    }
    return tables;
}
