#ifndef MAP_H
#define MAP_H
#include <stdio.h>
#include <limits.h>

#define MAP_SIZE 100
#define ROBOT_SPEED 20


extern int** map;
extern int coordonne_robot[2];
extern int direction_robot[2];
extern int robot_path[MAP_SIZE * MAP_SIZE][2];
extern int steps;
extern int EVASION_MODE; // New: Flag for evasion state

typedef enum {
    AVANCER,
    RECULER,
    TOURNER_GAUCHE,
    TOURNER_DROITE,
    ARRET_URGENCE
} type_mouvement;

typedef struct {
    type_mouvement type;
    float duree;
    float vitesse;
    int priorite;
} command;

// Queue declaration
typedef struct element {
    command* content;
    struct element* next;
} element;

typedef struct {
    element *head, *tail;
    int nb_elements;
} queue;

typedef struct Case {
    int val;                 // 0 = obstacle, 1 = road, 2 = forbidden, 3 = start, 4 = finish, 5 = path, 6 = robot
    struct Case* voisins[4]; // pointers to the 4 neighbors
    int g;                   // cost from start
    int h;                   // Manhattan heuristic
    int f;                   // f = g + h
    int visited;             // indicator if the case has been explored
    struct Case* parent;     // pointer to the parent for path reconstruction
    int i, j;
} Case;

// Priority queue for A*
typedef struct {
    Case* items[MAP_SIZE * MAP_SIZE];
    int size;
} PriorityQueue;

typedef struct {
    unsigned int r, g, b;
} pixel;

typedef struct {
    unsigned int rows, cols;
    pixel **pixels;
} image;

// --- map creating Functions ---
int in_bounds(int y, int x);
int able_proceed(int a, int d, int y_c, int x_c, int a_c, int d_c);
int random_length(int *pos, int *a_d);
void build_road(int **map, int *pos, int *cur_a_d, int *pre_a_d, int l);
int create_image(const char *nom_map, int **map);
int** create_map();
void destroy_map(image *map);

// --- Queue Functions ---
queue *queue_new();
void queue_free(queue *q);
int queue_send(queue *q, command *comm);
command *queue_receive(queue *q);
int queue_is_empty(queue *q);
unsigned queue_size(queue *q);
void ajouter_commande_normale(type_mouvement t, int d, float v, int p, queue *q);
void ajouter_commande_urgente(type_mouvement t, int d, float v, int p, queue *q);
const char* mouvementToString(type_mouvement t);
void executerCommande(command *c, FILE *journal);

// --- program principale ---
int move_robot(command *comm);
void prendre_prochaine_commande(queue* u, queue* n, FILE* j);
void rotate_to(int cur, int desired_dir, queue *n);
int step_forward(queue* n);
int check_robot_collision(int center_y, int center_x, int **map);
int check_front_sensor(int **map);
int evade_obstacle(int **map, int waypoints[MAP_SIZE*MAP_SIZE][2], int *current_wp_index, int n_waypoints, queue *n, queue *u, FILE *journal);

// --- A* Pathfinding Functions ---
void pq_init(PriorityQueue* pq);
int pq_contains(PriorityQueue* pq, Case* c);
void pq_push(PriorityQueue* pq, Case* c);
Case* pq_pop_min(PriorityQueue* pq);
int h_manhattan(Case* a, Case* b);
void construireGrille(Case grille[MAP_SIZE][MAP_SIZE], int** map);
int trouverDepartArrivee(Case** start, Case** goal, Case grille[MAP_SIZE][MAP_SIZE]);
int astar(Case* start, Case* goal);
void marquerChemin(Case* start, Case* goal, int** map);
int remplir_Chemin(Case* start, Case* goal, int path[MAP_SIZE*MAP_SIZE][2]);
int move_to_waypoint(int wp[2], queue *n, queue *u, FILE* f);
int follow_waypoints_grid(int waypoints[MAP_SIZE*MAP_SIZE][2], int n_waypoints, queue *n, queue *u, FILE* f);
#endif
