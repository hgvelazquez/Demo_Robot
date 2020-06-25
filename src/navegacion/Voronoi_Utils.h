static int LIBRE = 0;  /* No ha sido metido a la cola. */
static int OBSTACULO = 100;      /* Lo metimos una vez a la cola. */
static int ARISTA = 40;    /* Ya lo expandimos. */
static int NODO = 80;   /* Ya lo procesamos. */

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

