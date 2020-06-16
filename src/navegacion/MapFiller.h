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
 
const int WIDTH = 27;           // A lo largo del eje rojo x --- 15/0.3 = 4.5 mts
const int HEIGHT = 30;          // A lo largo del eje verde y -- 18/0.3 = 5.4 mts  
const float RESOLUTION = 0.2;   // metros/cuadro

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

/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
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

void fillOneRectangle(char* data, int i, int j, int value, int* obstacles, int obstacle, bool fill)
{
  data[j*WIDTH+i] = value;
  if (data[j*WIDTH+i] != 100)
        data[j*WIDTH+i] = value;
  if (value == 100 && fill)
    obstacles[j*WIDTH + i] = obstacle;
}


char* fillMap(int* obstacles, int size) {
  
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
  fillOneRectangle(data, (FRONT_TABLE_X_END)/2 + 1, FRONT_TABLE_Y_START, 100, obstacles, obstacle_count, true);
  fillOneRectangle(data, (FRONT_TABLE_X_END)/2 + 1, HEIGHT-2, 100, obstacles, obstacle_count++, true);
  
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
  fillRectangle(data, (CENTRAL_X_START+CENTRAL_X_END)/2, CENTRAL_Y_START, 1+(CENTRAL_X_START+CENTRAL_X_END)/2, CENTRAL_Y_START,100, obstacles, obstacle_count++, true);
  fillRectangle(data, (CENTRAL_X_START+CENTRAL_X_END)/2, CENTRAL_Y_END,  1+(CENTRAL_X_START+CENTRAL_X_END)/2, CENTRAL_Y_END, 100, obstacles, obstacle_count++, true);
  
  // Centrales
  fillOneRectangle(data, CENTRAL_X_START, (CENTRAL_Y_START + CENTRAL_Y_END)/2, 100, obstacles, obstacle_count++, true);
  fillOneRectangle(data, CENTRAL_X_END, (CENTRAL_Y_START + CENTRAL_Y_END)/2, 100, obstacles, obstacle_count++, true);
  fillRectangle(data, (CENTRAL_X_START+CENTRAL_X_END)/2, (CENTRAL_Y_START + CENTRAL_Y_END)/2, 1+ (CENTRAL_X_START+CENTRAL_X_END)/2, (CENTRAL_Y_START + CENTRAL_Y_END)/2, 100, obstacles, obstacle_count++, true);
  
  return data;
}

