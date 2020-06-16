#include <vector>
#include <queue>
#include <stack>

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

class Vertice{
  public:
    int x;
    int y;
    std::vector<Vecino*> vecinos;
    int color; // Para nuestro problema específico, 0 indica arista, 1 nodo de Voronoi...
    int visitado;
    Vertice* padre;
    
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
          int x1 = (current->x);
          int x2 = (vecinos[i]->vec->x);
          int y1 = (current->y);
          int y2 = (vecinos[i]->vec->y);
          Vertice* vor = vecinos[i]->vec->padre;
          float d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
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
