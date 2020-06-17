#include <vector>
#include <queue>
#include <stack>
#include <set>
#include <iterator> 
#include <iostream> 
#include <math.h>

static int SIN_VISITAR = 0;  /* No ha sido metido a la cola. */
static int EN_COLA = 1;      /* Lo metimos una vez a la cola. */
static int EXPANDIDO = 2;    /* Ya lo expandimos. */
static int FINALIZADO = 3;   /* Ya lo procesamos. */

/* Lo separé de esta forma pues quiero poder expandir antes de tiempo 
 *  a vértices que ya haya metido a la cola, puesto que los que son aristas
 *  serán metidos a la cola y para detectar al vértice que las maetió necesitan 
 *  expandirse la segunda vez que se metan. Además son las únicas que se meten 
 *  dos veces. 
 */

class Vecino;
class Vertice;
// Función para construir la gráfica sólo con los vértices de Voronoi.
std::vector<Vertice*> prune(std::vector<Vertice*> vertices, int first);
// Función para hacer la búsqueda A*.
std::stack<Vertice*> AStar(std::vector<Vertice*> vertices, Vertice* inicio, Vertice* meta);


class Vertice{
  public:
    int x;
    int y;
    std::vector<Vecino*> vecinos;
    int color; // Para nuestro problema específico, 0 indica arista, 1 nodo de Voronoi...
    int visitado;
    Vertice* padre;
    float f;
    float h;
    float g;
    
    Vertice(int x1, int y1) {
      x = x1;
      y = y1;
      visitado = 0;
      color = 0;
    }
};

class Vecino{
  public:
    Vertice* vec;
    float distancia;
  
    Vecino(Vertice* v, float d){
      vec = v;
      distancia = d;
    }  
    
    Vecino(Vertice* v){
      vec = v;
      distancia = 0;
    } 
};

float distancia(Vertice* v1, Vertice* v2) {
    int x1 = (v1->x);
    int x2 = (v2->x);
    int y1 = (v1->y);
    int y2 = (v2->y);
    float d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    return d;
}


std::vector<Vertice*> prune(std::vector<Vertice*> vertices, int first) {
  
  /** 
   * Este algoritmo se corre una única vez sobre la gráfica.
   * Por esto, asumimos que la bandera visitada es 0 para todo
   * vértice al iniciar el algoritmo.
   */
  
  std::vector<Vertice*> prunedGraph;
  std::stack<Vertice*> expanded;
  std::stack<Vertice*> currentVoronoi;
  Vertice* inicial = vertices[first];
  
  // Hay que inicializar un vértice nuevo para la gráfica de Voronoi.
  Vertice* inicialVoronoi = new Vertice(inicial->x, inicial->y);
  inicial->visitado = 1;
  currentVoronoi.push(inicialVoronoi);
  inicial->padre = inicialVoronoi;
  expanded.push(inicial);
  
  while(!expanded.empty()) {
    
    Vertice* current = currentVoronoi.top();
    Vertice* aux = expanded.top();
    
    int isVoronoi = (aux->color);
    int estadoVisita = (aux->visitado);
    
    //printf("TOP OF DFS STACK: x:%d, y:%d", aux->x, aux->y);
    //printf("TOP OF VORONOI STACK: x:%d, y:%d\n", current->x, current->y);
    
    // Ya regresamos, hay que procesar. 
    if (estadoVisita == EXPANDIDO) {
        //printf("POPPING FROM EXPANDED\n");
        expanded.pop(); // Lo sacamos sin expandir.
        if (aux->x == current->x && aux->y == current->y) {
          //printf("POPPING FROM VORONOI\n");
          prunedGraph.push_back(current);
          currentVoronoi.pop();
        } 
    } 
    // Su estado es 1, hay que meter a sus vecinos y cambiar su estado.
    else {
      std::vector<Vecino*> vecinos = (aux->vecinos);
      int size = vecinos.size();
      for (int i = 0; i < size; i++) {
        // Si no ha ido visitado, o está en la cola sin expandir. 
        if ((vecinos[i] -> vec -> visitado) == SIN_VISITAR || ((vecinos[i] -> vec -> visitado) == EN_COLA)) {
          // Si es de Voronoi.
          if ((vecinos[i]->vec->color) == 1) {
            int x1 = (current->x);
            int x2 = (vecinos[i]->vec->x);
            int y1 = (current->y);
            int y2 = (vecinos[i]->vec->y);
            Vertice* vor = new Vertice(x2, y2);
            currentVoronoi.push(vor);
            vecinos[i]->vec->padre = vor;
            float d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
            Vecino* vec1 = new Vecino(vor, d);
            Vecino* vec2 = new Vecino(current, d);
            (vor->vecinos).push_back(vec2);
            (current->vecinos).push_back(vec1);
          }
          (vecinos[i]->vec->visitado) = EN_COLA; 
          expanded.push(vecinos[i]->vec);
        } else if (vecinos[i]->vec->color == 1 && (vecinos[i]->vec->x != current->x || vecinos[i]->vec->y != current->y)) {
          Vertice* vor = vecinos[i]->vec->padre;
          float d = distancia(current, vecinos[i]->vec);
          Vecino* vec1 = new Vecino(vor, d);
          Vecino* vec2 = new Vecino(current, d);
          (current->vecinos).push_back(vec1);
          (vor->vecinos).push_back(vec2);
        }
      }
      aux->visitado = EXPANDIDO;
    }
  }
  return prunedGraph;  
}

std::stack<Vertice*> AStar(std::vector<Vertice*> vertices, Vertice* inicio, Vertice* meta) {
  auto cmp = [](Vertice* a, Vertice* b) { return (a->f) < (b->f);};
  /* Usamos un set ordenado pues la cola de prioridades de la biblioteca estándar 
   * no permite el acceso a los elementos, por lo que no podemos reordenar al 
   * elemento al que le cambiemos el valor de f. En cambio, con el conjunto, 
   * podemos acceder a los métodos erase e insert para sacarlo, cambiar su 
   * valor y después reinsertar. Lo que perdemos es que toma O(log n) sacar el mímimo.
   */
  // Lista Abierta
  std::set<Vertice*, decltype(cmp)> listaAbierta(cmp);
  /* Lista cerrada, aún lo necesitamos aunque la abierta sea conjunto pues a la abierta 
   " le eliminamos elementos.*/ 
  std::set<Vertice*> listaCerrada;
  int size = vertices.size();
  for (int i = 0; i < size; i++) {
      float d = distancia(vertices[i], meta);
      vertices[i] -> h = d;
      vertices[i] -> f = FLT_MAX;
  }
  
  inicio->g = 0;
  inicio->f = (inicio->g) + (inicio->h);
  inicio->padre = NULL;
  
  meta->h = 0;
  listaAbierta.insert(inicio);
  while (!listaAbierta.empty()) {
    Vertice* actual = (*listaAbierta.begin());
    listaAbierta.erase(actual);
    listaCerrada.insert(actual);
     
    if (actual == meta)
      break;
      
    std::vector<Vecino*> vecinos = actual->vecinos;
    size = vecinos.size();
    for (int i = 0; i<size; i++) {
      Vertice* sucesor = vecinos[i]->vec;
      float g = (actual->g) + distancia(actual, sucesor);
      if (listaCerrada.count(sucesor) == 0) {
        if (listaAbierta.count(sucesor) == 0) {
          sucesor->padre = actual;
          sucesor->g = g;
          sucesor->f = (sucesor->g) + (sucesor->h);
          listaAbierta.insert(sucesor);
        } else if (g < sucesor->g) {
          listaAbierta.erase(sucesor);
          sucesor->g = g;
          sucesor->f = (sucesor->g) + (sucesor->h);
          sucesor->padre = actual;
          listaAbierta.insert(sucesor);
        }
      }
    }
  }
  
  std::stack<Vertice*> ruta;
  Vertice* padre = meta;
  while(padre != NULL) {
      ruta.push(padre);
      padre = padre->padre;
  }   
  return ruta;
}

