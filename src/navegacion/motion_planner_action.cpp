/* Nodo que se encarga de ir mandando las metas intermedias para mover al robot.*/
#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

#include <vector>
#include <queue>
#include <stack>
#include <string>
#include <math.h>

#include "Grafica.h"
#include "MapFiller.h"
#include "Voronoi_Utils.h"
#include "../LogHeader.h"

using std::vector;

class NavigationAction
{
protected:
  
    /* Variables necesarias para implementar una acción. */
    ros::NodeHandle nh_ = ros::NodeHandle("~");
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    move_base_msgs::MoveBaseFeedback feedback_;
    move_base_msgs::MoveBaseResult result_;



    /* Variables para construir y guardar la gráfica de Voronoi. */
    // El mapa con la información de obstáculos.
    nav_msgs::OccupancyGrid MAP;
    // La gráfica de Voronoi. 
    std::vector<Vertice*> VORONOI_GRAPH;
    // El vértice de Voronoi más cercano a cada celda.
    std::vector<Vertice*> nearest_voronoi;

    /* Variables ocupadas para los cálculos de las metas. */
    geometry_msgs::Point goal;  /* La meta final del movimiento. */
    geometry_msgs::Point next;  /* La siguiente meta de la ruta. */
    int sent_goal;              /* Si ya enviamos la meta (no está en la ruta de A*).*/
    float origin_x;             /* El origen en x del mapa (para lectura sencilla). */
    float origin_y;             /* El origen en y del mapa (para lectura sencilla). */
    float resolution;           /* La resolución del mapa (para lectura sencilla). */
    int begin_turn;             /* Nos dice si hay que iniciar a girar .*/
    int turn;                   /* Nos dice si ya giramos.*/
    

    /* Varables para las metas intermedias. */
    std::stack<Vertice*> a_star_route;  /* La pila con las metas de la ruta. */
    geometry_msgs::Pose kob_pose;       /* La posición actual de la KOBUKI. */
    bool almost;                        /* Nos indica si llegamos aproximadamente a la meta intermedia. */
    bool reached_last;                  /* Nos dice si ya llegamos a la última intermedia.*/

    /* Variable que nos indica que ya tenemos la información del mapa y podemos dejar de ecuchar. */
    bool received_map;  

    /* Variable que utilizamos para obtener los parametros. */
    string m_robot_name;
    double INITIAL_X;
    double INITIAL_Y;
    int DEBUG;
    LogHeader m_logheader;

    /* Tópicos donde publicaremos. */
    ros::Publisher a_star_pub;
    ros::Publisher rotate_pub;
    ros::Publisher finished_pub;
    ros::Publisher path_pub;
    
    /* Tópicos donde escucharemos. */
    ros::Subscriber nav_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;

    /** 
     * Función callback para el tópico del mapa. 
     * En cuanto ha recibido un mapa válido, lo anuncia con
     * received_map, para poder dejar de escuchar el tópico después de esto
     * (ya que el mapa no está diseñado para cambiar).
     */
    void getMapParams(const  nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        MAP = *msg; 
        origin_x = MAP.info.origin.position.x;
        origin_y = MAP.info.origin.position.y;
        resolution = MAP.info.resolution;
        received_map = true;
    }

    /**
     * Función auxiliar para obtener el punto real (no como celda) 
     * dado un vértice. 
     */
    geometry_msgs::Point getPoint(Vertice* v) {
        geometry_msgs::Point p; 
        p.x = (v->x)*RESOLUTION + origin_x;
        p.y = (v->y)*RESOLUTION + origin_y;
        return p;
    }

    /**
     * Función callback para el odómetro.
     * Se encarga de actualizar la variable kob_pose, y adicionalmente, 
     * dependiendo de la distancia a la que se encuentra de la meta 
     * siguiente, actualiza almost, lo cual nos dará la señal de 
     * cambiar a la siguiente meta.
     */
    void updatePoint(const nav_msgs::Odometry::ConstPtr& msg){
        nav_msgs::Odometry od = *msg;
        kob_pose = od.pose.pose;
        kob_pose.position.x += INITIAL_X;
        kob_pose.position.y += INITIAL_Y;
        m_logheader.debugOnce("x: "+to_string(kob_pose.position.x)+" y: "+to_string(kob_pose.position.y));
        
        // Si ya enviamos la meta, no tenemos que actualizar nada ya. 
        if (sent_goal == 0) {
            float dx = kob_pose.position.x - next.x;
            float dy = kob_pose.position.y - next.y;
            float d = sqrt(dx*dx + dy*dy);
            almost = d < 0.2;            
            if (!a_star_route.empty() && almost) {
                a_star_route.pop();
                if (a_star_route.empty()) {
                    reached_last = true;
                } else {
                    next = getPoint(a_star_route.top());
                    printf("NEXT: %d, %d\n", a_star_route.top()->x, a_star_route.top()->y);
                }
                almost = false;
            }
        } else {
            float dx = kob_pose.position.x - goal.x;
            float dy = kob_pose.position.y - goal.y;
            float d = sqrt(dx*dx + dy*dy);
            begin_turn = d < 0.1;  
        } 
    }

    /**
     * Función auxiliar para expandir las celdas en el algoritmo de 
     * expansión de ondas para encontrar los vértices de Voronoi más 
     * cercanos a cada celda.
     * Recibe a la celda a expandir, c; el que lo intenta expandir, padre,
     * la cola a la que se mete si se expande, el vector con los vértices
     * más cercanos (detectados hasta ese punto) y un booleano que nos
     * dice si estamos intentado expandir en diagonal.
     */
    void processCellVoronoi(Celda& c, Celda& padre,  
                            std::priority_queue<Celda, std::vector<Celda>, compareCelda>& cola, 
                            std::vector<Vertice*>& nearest_vor, bool diag) 
    {
    int diff = (diag) ? 3 : 2;
    if (c.lleno > padre.lleno + diff) {
        c.lleno = padre.lleno + diff;
        c.vertice_vor = padre.vertice_vor;
        nearest_vor[c.x + c.y*MAP.info.width] = c.vertice_vor;
        cola.push(c);
    }  
    }

    /**
     * Función auxiliar que se encarga de agregar un vecino a un vértice
     * en la gráfica de voronoi. 
     * Se agrega el vecino al vértice v si y sólo si, se había detectado que 
     * la celda (del grid) correspondiente a la celda p era parte de la gráfica
     * de voronoi (incluyendo celdas aristas, por lo que después hay que 
     * buscar a los vecinos que si son celdas nodo).
     * Regresa 1 si añade el vecino, 0 en otro caso. 
     */
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

    /**
     * 
     * 
     */
    nav_msgs::OccupancyGrid get_Astar_graph(nav_msgs::OccupancyGrid MAP, int src_x, int src_y, int dest_x, int dest_y){
        // Obtenemos la información del mapa para fácil escritura.  
        int HEIGHT = MAP.info.height;
        int WIDTH = MAP.info.width;
        vector<int8_t> data = MAP.data; 
        vector<bool> visited (MAP.data.size(),false); 
        vector<int> dist_src (MAP.data.size(),0);
        vector<int> h_dist_dest (MAP.data.size(),0);
        vector<int> f_sum_dist (MAP.data.size(),0);

        m_logheader.info("src: x: "+to_string(src_x)+" y: "+to_string(src_y));
        m_logheader.info("src: x: "+to_string(dest_x)+" y: "+to_string(dest_y));

        data[src_x + src_y*WIDTH];
        // string s = "";

        // for(int i = 0; i < HEIGHT; i++){
        //     for (int j = 0; j < WIDTH; j++){
        //         s += to_string(data[i + j*WIDTH]);
        //     }
        //     s += "\n";
        // }


        MAP.data = data;
        
        return MAP;
    } 

    /**
     * Función para cpnstruir la gráfica de VORONOI.
     * y obtiene los vértices más cercanos a cada celda. 
     * Ambos se ponen aquí pues requieren de los datos de una
     * matriz, y en C++ no se puede pasar como parámetro a menos
     * que se conozcan las dimensiones a tiempo de compilación. 
     * Actualiza tanto nearest_voronoi y VORONOI_GRAPH.
     */
    void construyeVoronoi() 
    {
        // Obtenemos la información del mapa para fácil escritura.  
        int HEIGHT = MAP.info.height;
        int WIDTH = MAP.info.width;
        std::vector<int8_t> data = MAP.data; 
    
        Celda info[WIDTH][HEIGHT];
        // Gráfica inicial (con nodos aristas incluidas)
        std::vector<Vertice*> grafica_voronoi;
        // Cola para expandir en el algoritmo de voronoi más cercano.
        std::priority_queue<Celda, std::vector<Celda>, compareCelda> cola;
        // Vector con el vértice de voronoi más cercano a cada celda. 
        std::vector<Vertice*> nearest_vor(WIDTH*HEIGHT);

        // Llenamos con los datos iniciales. 
        for (int j = 0; j < HEIGHT; j++) {
            for (int i = 0; i < WIDTH; i++) {
            // Creamos la celda, le ponemos sus coordenadas y llenamos con 
            // un valor muy alto
            Celda celda;
            info[i][j] = celda;
            info[i][j].x = i;
            info[i][j].y = j;
            info[i][j].lleno = 2*HEIGHT*WIDTH;
            int type = MAP.data[i + j*WIDTH];
            // Si es parte de grafica_voronoi...
            if (type == NODO || type == ARISTA){
                // Creamos el vértice
                Vertice* v = new Vertice(i, j);
                grafica_voronoi.push_back(v);
                info[i][j].vertice_vor = v;
                if (type == ARISTA) {
                    info[i][j].seleccionado = 1;
                } else {
                    // Si es nodo, lo metemos para expandir en el algoritmod de más cercano.
                    info[i][j].lleno = 0;
                    info[i][j].seleccionado = 2;
                    info[i][j].expandido = 1;
                    cola.push(info[i][j]);
                }        
            } else if (type == OBSTACULO) {
                // Si es obstáculo, marcamos que no importa para expandir. 
                info[i][j].lleno = -1;
                nearest_vor[i + j*WIDTH] = new Vertice(i, j);
            }
        }     
        }
    
        /**
         * Algoritmo de construcción de la gráfica de Voronoi.
         */
        int l = grafica_voronoi.size();
        int first = -1;
        // Encontramos a aquellos que son vértices de voronoi y construimos la gráfica a recortar. 
        for (int i = 0; i < l; i++){
            Vertice* actual = grafica_voronoi[i];
            int x = actual->x;
            int y = actual->y;
        
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
                info[x][y].seleccionado = 2;
                (actual->color) = 1;
                if (first == -1) {
                    first = i;
                }  
            }  
        }  
    
        // Obtenemos la gráfica de Voronoi final.
        VORONOI_GRAPH = prune(grafica_voronoi, first);
        l = VORONOI_GRAPH.size();
        // Imprimimos la gráfica de Voronoi.
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
    
        /** Algoritmo de expansión de onda para encontrar los 
         * vértices más cercanos.
         */
        // Indicamos que los más cercanos a las celdas que son nodos son ellas mismas. 
        for (int i = 0; i<l; i++) {
            Vertice* v = VORONOI_GRAPH[i];
            info[v->x][v->y].vertice_vor = v;
            nearest_vor[(v->x) + (v->y)*WIDTH] = v;
        }
        
        int previ, prevj, nexti, nextj;
        Celda current;
        while(!cola.empty()) {
            current = cola.top();
            cola.pop();
            int i = current.x;
            int j = current.y;
            previ = i-1;
            prevj = j-1;
            nexti = i+1;
            nextj = j+1;
            int lleno = info[i][j].lleno + 1;
            if (i != 0 && data[previ + j*WIDTH] != OBSTACULO)
                processCellVoronoi(info[previ][j], info[i][j], cola, nearest_vor, false);
            if (j != 0 && data[i + prevj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[i][prevj], info[i][j], cola, nearest_vor, false);     
            if (i!= WIDTH - 1 && data[nexti + j*WIDTH] != OBSTACULO)
                processCellVoronoi(info[nexti][j], info[i][j], cola, nearest_vor, false);    
            if (j != HEIGHT-1 && data[i + nextj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[i][nextj], info[i][j], cola, nearest_vor, false);
            if (i != 0 && j != 0 && data[previ + prevj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[previ][prevj], info[i][j], cola, nearest_vor, true);
            if (i != WIDTH -1 && j != 0 && data[nexti + prevj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[nexti][prevj], info[i][j], cola, nearest_vor, true);
            if (i != WIDTH - 1 && j != HEIGHT -1 && data[nexti + nextj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[nexti][nextj], info[i][j], cola, nearest_vor, true);
            if (i != 0 && j != HEIGHT-1 && data[previ + nextj*WIDTH] != OBSTACULO)
                processCellVoronoi(info[previ][nextj], info[i][j], cola, nearest_vor, true);
        }
    
        /**
     
        for (int i = 0; i<WIDTH; i++) {
            for (int j = 0; j<HEIGHT;j++) {
            printf("CELL x:%d, y:%d, NEAREST x:%d, y:%d\n", i, j, nearest_vor[j*WIDTH+i]->x, nearest_vor[j*WIDTH+i]->y);
            }  
        }*/
    
        nearest_voronoi = nearest_vor;
    }

    /**
     * Función que rota a la KOBUKI hasta que quede viendo defrente al
     * objetivo original.
     */
    geometry_msgs::Twist rotate() 
    {
        geometry_msgs::Twist vel;
        // Creamos el cuaternión para rotar a las coordenadas adecuadas.
        tf2::Quaternion v1(kob_pose.orientation.x, kob_pose.orientation.y, kob_pose.orientation.z, kob_pose.orientation.w);
        
        // Pasamos el goal al marco de la KOBUKI.
        geometry_msgs::Point goal_coord;
        // Trasladamos.
        goal_coord.x = goal.x - kob_pose.position.x;
        goal_coord.y = goal.y - kob_pose.position.y;
        // Rotamos.
        tf2::Quaternion g(goal_coord.x, goal_coord.y, 0, 0);
        g = inverse(v1)*g*v1;
        goal_coord.x = g.x();
        goal_coord.y = g.y();
        
        // Vectores para calcular el ángulo entre el goal y el frente de la KOBUKI.
        tf2::Vector3 vg(goal_coord.x, goal_coord.y, 0);
        tf2::Vector3 vk(1, 0, 0);
        float a = vk.angle(vg);
        if (a > 5*M_PI/180) { // Margen de error de 5 grados.
            float factor = (a < M_PI/3) ? 2 : 1; 
            int sign = (goal_coord.y > 0) ? 1 : -1;
            vel.angular.z = factor*sign*a;
        } else {
            turn = 1;
        }
        return vel;
    }



    /**
     * Función callback para recibir la meta del movimiento.
     * actualiza las variables goal, sent_goal y a_star_route.
     * La última de estas la actualiza haciendo el cálculo de 
     * la ruta con A*. 
     */
    void receiveNavGoal(const geometry_msgs::PoseStamped::ConstPtr& poseStamped)
    {
        m_logheader.debug("Received Goal!");

        // Sacamos el ancho para escribirlo más fácilmente.
        int WIDTH = MAP.info.width;
        
        goal = (*poseStamped).pose.position;
        sent_goal = 0;
        
        // Obtenemos la celda en la que se encuentra la KOBUKI.
        int u = floor((kob_pose.position.x - origin_x)/RESOLUTION);
        int w = floor((kob_pose.position.y - origin_y)/RESOLUTION);
        // ROS_INFO(m_logheader.getHeader("YOU ARE AT: %d, %d\n", u, w).c_str());
        m_logheader.debug("YOU ARE AT x:"+to_string(u)+", y:"+to_string(w));
        
        // Obtenemos la celda a la que queremos llegar. 
        int x = floor((goal.x - origin_x)/RESOLUTION);
        int y = floor((goal.y - origin_y)/RESOLUTION);
        // ROS_INFO(m_logheader.getHeader("GOING TO : %d, %d\n", x, y).c_str());
        m_logheader.debug("GOING TO x:"+to_string(x)+", y:"+to_string(y));

        
        // Obtenemos los vértices más cerecanos a ambas celdas.
        Vertice* inic = nearest_voronoi[u + w*WIDTH];
        Vertice* fin = nearest_voronoi[x + y*WIDTH];
        
        
        // ROS_INFO(m_logheader.getHeader("INIC: %d, %d\n", inic->x, inic->y).c_str());
        // ROS_INFO(m_logheader.getHeader("%2.3f, %2.3f\n", getPoint(inic).x, getPoint(inic).y).c_str());
        // ROS_INFO(m_logheader.getHeader("END:  %d, %d\n", fin->x, fin->y).c_str());
        // ROS_INFO(m_logheader.getHeader("%2.3f, %2.3f\n", getPoint(fin).x, getPoint(fin).y).c_str());
        // Obtenemos la ruta en la gráfica con A*.
        std::stack<Vertice*> a = AStar(VORONOI_GRAPH, inic, fin); 

        a_star_route = a;
        next = getPoint(a_star_route.top());
        
        // ROS_INFO(m_logheader.getHeader("NEXT: %d, %d\n", a_star_route.top()->x, a_star_route.top()->y).c_str());
        almost = false;
        reached_last = false;
        turn = 0;
    }

    void readParams(string name){
        m_robot_name = "";
        INITIAL_X = 0;
        INITIAL_Y = 0;
        DEBUG = 0;
        if (!nh_.hasParam("robot_name"))
            m_logheader.error("No param named 'robot_name'");

        if (nh_.getParam("robot_name", m_robot_name)){
            m_logheader.info("Got param: "+m_robot_name);
            action_name_ = "/"+m_robot_name+"/"+name;
            m_logheader.m_robot_id = m_robot_name;
        }
        else {
            m_logheader.error("Failed to get param 'robot_name'");
            action_name_ = "/"+name;
        }

        if (nh_.getParam("x", INITIAL_X))
            m_logheader.info("Got param "+to_string(INITIAL_X));
        else
            m_logheader.error("Failed to get param 'x'");

        if (nh_.getParam("y", INITIAL_Y))
            m_logheader.info("Got param "+to_string(INITIAL_Y));
        else
            m_logheader.error("Failed to get param 'y'");

        if (nh_.getParam("debug", DEBUG))
            m_logheader.info("Got param %d"+to_string(DEBUG));
            if(DEBUG){
                if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
                    ros::console::notifyLoggerLevelsChanged();
                }
            }
        else
            m_logheader.error("Failed to get param 'debug'");
    }

public:

    NavigationAction(std::string name){
        m_logheader.m_file = "Motion_planner_action.cpp";
        // Recibimos los argumentos para obtener el nombre del robot
        readParams(name);

        as_ = new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(ros::NodeHandle(), action_name_, boost::bind(&NavigationAction::executeCB, this, _1), false);
        as_->start();
        

        // Tópicos donde publicaremos.
        a_star_pub = nh_.advertise<geometry_msgs::Point>("a_star_goal", 5);
        rotate_pub = nh_.advertise<geometry_msgs::Twist>("/dummy/mobile_base/commands/velocity", 1);
        path_pub = nh_.advertise<nav_msgs::OccupancyGrid>("path_info", 1);
        
        // Tópicos donde escucharemos.
        nav_sub = nh_.subscribe<geometry_msgs::PoseStamped>(("/"+m_robot_name+"/move_base_simple/goal").c_str(), 5, boost::bind(&NavigationAction::receiveNavGoal, this, _1));
        odom_sub = nh_.subscribe<nav_msgs::Odometry>("odom", 5, boost::bind(&NavigationAction::updatePoint, this, _1));
        map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/voronoi_info", 5, boost::bind(&NavigationAction::getMapParams, this, _1));

        m_logheader.info("suscribed to: /"+m_robot_name+"/move_base_simple/goal");
        m_logheader.info("suscribed to: /"+m_robot_name+"/odom");

        ros::Rate r(10);
        received_map = false;
        // Esperamos hasta recibir el mapa por primera vez...
        while(ros::ok() && !received_map)
        {
            ros::spinOnce();
            r.sleep();
        }
        // Y ya no lo recibimos más (sólo lo necesitamos una vez, no cambia).
        map_sub.shutdown();
        
        construyeVoronoi();
    }

    ~NavigationAction(void)
    {
    }


    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& moveBaseGoal)
    {
        m_logheader.info("receivedGoal!");
        // Sacamos el ancho para escribirlo más fácilmente.
        int WIDTH = MAP.info.width;
        


        goal = moveBaseGoal->target_pose.pose.position;
        sent_goal = 0;
        
        // Obtenemos la celda en la que se encuentra la KOBUKI.
        int u = floor((kob_pose.position.x - (origin_x))/RESOLUTION);
        int w = floor((kob_pose.position.y - (origin_y))/RESOLUTION);
        // ROS_INFO("Motion_planner_action.cpp [%s] YOU ARE AT: %d, %d\n", m_robot_name.c_str(), u, w);
        m_logheader.info("YOU ARE AT (world frame) x:"+to_string(kob_pose.position.x)+", y:"+to_string(kob_pose.position.y));
        m_logheader.info("YOU ARE AT (OG frame) x:"+to_string(u)+", y:"+to_string(w));
        
        // Obtenemos la celda a la que queremos llegar. 
        int x = floor((goal.x - (origin_x))/RESOLUTION);
        int y = floor((goal.y - (origin_y))/RESOLUTION);
        // ROS_INFO("Motion_planner_action.cpp [%s] GOING TO : %d, %d\n", m_robot_name.c_str(),x, y);
        m_logheader.info("YOU ARE GOING TO x:"+to_string(x)+", y:"+to_string(y));
        
        path_pub.publish(get_Astar_graph(MAP, u, w, x, y));

        // Obtenemos los vértices más cerecanos a ambas celdas.
        Vertice* inic = nearest_voronoi[u + w*WIDTH];
        Vertice* fin = nearest_voronoi[x + y*WIDTH];
        
        
        // ROS_INFO("motion_planner_action.cpp [%s] INIC: %d, %d\n", m_robot_name.c_str(), inic->x, inic->y);
        // ROS_INFO("motion_planner_action.cpp [%s] : %2.3f, %2.3f\n", m_robot_name.c_str(), getPoint(inic).x, getPoint(inic).y);
        // ROS_INFO("motion_planner_action.cpp [%s] END:  %d, %d\n", m_robot_name.c_str(), fin->x, fin->y);
        // ROS_INFO("motion_planner_action.cpp [%s] : %2.3f, %2.3f\n", m_robot_name.c_str(), getPoint(fin).x, getPoint(fin).y);
        // Obtenemos la ruta en la gráfica con A*.
        std::stack<Vertice*> a = AStar(VORONOI_GRAPH, inic, fin); 

        a_star_route = a;
        next = getPoint(a_star_route.top());
        
        string stmp = "NEXT: x: "+to_string(a_star_route.top()->x)+", y: "+to_string(a_star_route.top()->y);
        m_logheader.debug(stmp);
        almost = false;
        reached_last = false;

        // helper variables
        bool success = false;

        begin_turn = 0; /* Hasta que no tengamos que girar, no cambiamos esto.*/
        turn = 0;
        ros::Rate r(10);
        
        // start executing the action
        while (ros::ok())
        {
            move_base_msgs::MoveBaseFeedback feedback;
            feedback.base_position.pose = kob_pose;
            // Mensaje de término. 
            ros::spinOnce();
            // Hay que agregar código para no republicar el punto...
            if (!a_star_route.empty() && !reached_last) {
                a_star_pub.publish(next);
            } else if (sent_goal == 0) {     
                m_logheader.debug("GOING TO FINAL GOAL!");
                a_star_pub.publish(goal);
                sent_goal = 1;
            } else if (begin_turn == 1 && turn == 0) {
                //    rotate_pub.publish(rotate());
                turn = 1;
            } else if (turn == 1){
                // Ya hicimos todo lo que teníamos que hacer. 
                success = true;
                break;
            }    
            as_->publishFeedback(feedback);
            r.sleep();
        }
        
        if(success)
        {
            m_logheader.info("Succeeded!");
            // set the action state to succeeded
            as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        }
    }

};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "motion_planner_action");  
    
    NavigationAction nav_act("motion_planner_action");
    ros::spin();

    return 0;
}
