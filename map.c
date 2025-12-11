#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "map.h"

// --- Global Variables Definition ---
int** map = NULL;
int coordonne_robot[2] = {95, 4}; // Start coordinates (Y, X)
int direction_robot[2] = {0, 1};  // Initial direction (0, 1) = East
int robot_path[MAP_SIZE * MAP_SIZE][2];
int steps = 0;
int EVASION_MODE = 0; 

int dir4[4][2] = {
    {-1, 0},  // 0: Nord
    { 0, 1},  // 1: Est
    { 1, 0},  // 2: Sud
    { 0,-1}   // 3: Ouest
};

int in_bounds(int y, int x) {
    return (y >= 0 && y < MAP_SIZE && x >= 0 && x < MAP_SIZE);
}

int able_proceed(int a, int d, int y_c, int x_c, int a_c, int d_c) {
    if (y_c < 9 && (a == 1 || d == 1)) return 0;

    if (a == 0 && a_c == 0) {
        if (d != d_c) return 0;
        else {
            if (d == 0) return  (x_c < MAP_SIZE - 5); 
            else return (x_c > 4);
        }
    }

    if (a == 1 && a_c == 1) {
        if (d != d_c) return 0;
        else {
            if (d == 0) return  (y_c > 4);
            else return (y_c < MAP_SIZE - 5);
        }
    }

    if (a == 0 && a_c == 1) {
        if (d == 0) return x_c < MAP_SIZE - 5;
        return x_c > 4;
    }
    if (a == 1 && a_c == 0) {
        if (d == 0) return y_c > 4;
        return y_c < MAP_SIZE - 5;
    }
    return 1;
}

int random_length(int pos[2], int a_d[2]) {
    int x = pos[1], y = pos[0];
    int a = a_d[0], d = a_d[1];
    if (y < 9) return (MAP_SIZE - 1) - x;
    int max;
    int min = 1;

    if (a == 0) {
        if (d == 0) max = (MAP_SIZE - 5) - x;
        else max = x - 4;
    }
    else {
        if (d == 0) max = y - 4;
        else max = (MAP_SIZE - 1) - y;
    }

    if (max < min) return min;
    int l = ((rand() % (max - min + 1)) + min);
    return l;
}

void build_road(int **map, int *pos, int *cur_a_d, int *pre_a_d, int l) {
    int x_c = pos[1], y_c = pos[0];
    int a = cur_a_d[0], d = cur_a_d[1];
    int p_a = pre_a_d[0];
    static int distance_obstacal = 5;
    int min_dimension_obst = 1;
    int max_dimension_obst = 2;

    if (a != p_a) {
        for (int y = y_c - 4; y < y_c + 5; y++) {
            if (y < 0 || y >= 100) continue;
            for (int x = x_c - 4; x < x_c + 5; x++) {
                if (x < 0 || x >= 100) continue;
                map[y][x] = 1;
            }
        }
    }

    if (a == 0 && d == 0) {
        for (int i = 0; i < l; i++) {
            pos[1] += 1;
            if (distance_obstacal == 0) {
                int put_obst = rand() % 2;
                if (put_obst) {
                    int x_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int y_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int pos_obst_road = (-2 * (rand() % 2) + 1) * ((rand() % (2)) + 2);
                    int pos_obst_y = y_c + pos_obst_road;
                    for (int y = pos_obst_y - 1; y < pos_obst_y + y_dim -1; y++) {
                        for (int x = pos[1] - 1; x < pos[1] + x_dim - 1; x++) {
                            map[y][x] = 0;
                        }
                    }
                    distance_obstacal += 1;
                }
            }
            else distance_obstacal += 1;

            if (distance_obstacal == 15) distance_obstacal = 0;

            for (int j = y_c - 4; j < y_c + 5; j++) {
                if (map[j][pos[1]] != 0) map[j][pos[1]] = 1;
            }
        }
    }
    if (a == 0 && d == 1) {
        for (int i = 0; i < l; i++) {
            pos[1] -= 1;
            if (distance_obstacal == 0) {
                int put_obst = rand() % 2;
                if (put_obst) {
                    int x_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int y_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int pos_obst_road = (-2 * (rand() % 2) + 1) * ((rand() % (2)) + 2);
                    int pos_obst_y = y_c + pos_obst_road;
                    for (int y = pos_obst_y - 1; y < pos_obst_y + y_dim -1; y++) {
                        for (int x = pos[1] - 1; x < pos[1] + x_dim - 1; x++) {
                            map[y][x] = 0;
                        }
                    }
                    distance_obstacal += 1;
                }
            }
            else distance_obstacal += 1;

            if (distance_obstacal == 9) distance_obstacal = 0;

            for (int j = y_c - 4; j < y_c + 5; j++) {
                if (map[j][pos[1]] != 0) map[j][pos[1]] = 1;
            }
        }
    }
    if (a == 1 && d == 0) {
        for (int i = 0; i < l; i++) {
            pos[0] -= 1;
            if (distance_obstacal == 0) {
                int put_obst = rand() % 2;
                if (put_obst) {
                    int x_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int y_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int pos_obst_road = (-2 * (rand() % 2) + 1) * (((rand() % (2)) + 2));
                    int pos_obst_x = x_c + pos_obst_road;
                    for (int y = pos[0] - 1; y < pos[0] + y_dim -1; y++) {
                        for (int x = pos_obst_x - 1; x < pos_obst_x + x_dim - 1; x++) {
                            map[y][x] = 0;
                        }
                    }
                    distance_obstacal += 1;
                }
            }
            else distance_obstacal += 1;

            if (distance_obstacal == 9) distance_obstacal = 0;

            for (int j = x_c - 4; j < x_c + 5; j++) {
                if (map[pos[0]][j] != 0) map[pos[0]][j] = 1;
            }
        }
    }
    if (a == 1 && d == 1) {
        for (int i = 0; i < l; i++) {
            pos[0] += 1;
            if (distance_obstacal == 0) {
                int put_obst = rand() % 2;
                if (put_obst) {
                    int x_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int y_dim = ((rand() % (max_dimension_obst - min_dimension_obst + 1)) + min_dimension_obst);
                    int pos_obst_road = (-2 * (rand() % 2) + 1) * (((rand() % (2)) + 2));
                    int pos_obst_x = x_c + pos_obst_road;
                    for (int y = pos[0] - 1; y < pos[0] + y_dim -1; y++) {
                        for (int x = pos_obst_x - 1; x < pos_obst_x + x_dim - 1; x++) {
                            map[y][x] = 0;
                        }
                    }
                    distance_obstacal += 1;
                }
            }
            else distance_obstacal += 1;

            if (distance_obstacal == 9) distance_obstacal = 0;

            for (int j = x_c - 4; j < x_c + 5; j++) {
                if (map[pos[0]][j] != 0) map[pos[0]][j] = 1;
            }
        }
    }
}

int** create_map() {
    // 0 ==> obstacle
    // 1 ==> road
    // 2 ==> unauthorized-zone
    // 3 ==> start
    // 4 ==> finish
    srand((unsigned) time(NULL));

    int **new_map = malloc(MAP_SIZE * sizeof(int*));
    for (int y = 0; y < MAP_SIZE; y++) {
        new_map[y] = malloc(MAP_SIZE * sizeof(int));
        for (int x = 0; x < MAP_SIZE; x++) {
            new_map[y][x] = -1;
        }
    }

    // 0 : axe des x; 1 : axe des y
    int axes[] = {0, 1};
    // 0 : sens croissant; 1 : sens decroissant
    int directions[] = {0, 1};
    int current_pos[] = {95, 4};
    int current_axe_direction[] = {0, 0};
    for (int y = 91; y < 100; y++) {
        for (int x = 0; x < 5; x++) {
            new_map[y][x] = 1;
        }
    }

    while (current_pos[0] > 12 || current_pos[1] != 99) {
        //sleep_ms(400);
        int available_directions[4][2];
        int direc_num = 0;
        if (able_proceed(0, 0, current_pos[0], current_pos[1], current_axe_direction[0], current_axe_direction[1])) {
            available_directions[direc_num][0] = 0;
            available_directions[direc_num][1] = 0;
            direc_num++;
        }
        if (able_proceed(0, 1, current_pos[0], current_pos[1], current_axe_direction[0], current_axe_direction[1])) {
            available_directions[direc_num][0] = 0;
            available_directions[direc_num][1] = 1;
            direc_num++;
        }
        if (able_proceed(1, 0, current_pos[0], current_pos[1], current_axe_direction[0], current_axe_direction[1])) {
            available_directions[direc_num][0] = 1;
            available_directions[direc_num][1] = 0;
            direc_num++;
        }
        if (able_proceed(1, 1, current_pos[0], current_pos[1], current_axe_direction[0], current_axe_direction[1])) {
            available_directions[direc_num][0] = 1;
            available_directions[direc_num][1] = 1;
            direc_num++;
        }

        if (direc_num == 0) {
            /* no more directions */
            break;
        }

        int rand_int = rand() % direc_num;
        int previous_a_d[2] = {current_axe_direction[0], current_axe_direction[1]};
        current_axe_direction[0] = available_directions[rand_int][0];     // ((rand() % (max - min + 1)) + min) with min = 0 and max = direc_num - 1
        current_axe_direction[1] = available_directions[rand_int][1];     // this formula generates a pseudo-random number between min and max

        int route_length = random_length(current_pos, current_axe_direction);
        build_road(new_map, current_pos, current_axe_direction, previous_a_d, route_length);
    }

    for (int y = 0; y < 9; y++) {
        for (int x = 91; x < 100; x++) {
            new_map[y][x] = 1;
        }
    }

    new_map[95][4] = 3;
    new_map[4][95] = 4;

    for (int y = 0; y < 100; y++) {
        for (int x = 0; x < 100; x++) {
            if (new_map[y][x] == -1) new_map[y][x] = 2;
        }
    }
    return new_map;
}

int create_image(const char *nom_map, int **map) {
    FILE *file = fopen(nom_map, "wb");
    if (file == NULL) return 0;
    fprintf(file, "P6\n%u %u\n%u\n", 100, 100, 255);

    pixel obstacle = {136, 0, 21};
    pixel road = {128, 128, 128};
    pixel unauthorized_zone = {24, 62, 12};
    pixel start_finish = {0, 0, 0};
    pixel undifined = {255, 242, 0};
    pixel path = {163, 73, 164};
    pixel robot = {181, 230, 29};

    for (int y = 0; y < 100; y++) {
        for (int x = 0; x < 100; x++) {
            pixel p;
            switch (map[y][x]) {
                case 0: p = obstacle; break;
                case 1: p = road; break;
                case 2: p = unauthorized_zone; break;
                case 3:
                case 4: p = start_finish; break;
                case 5: p = path; break;
                case 6: p = robot; break;
                default: p = undifined;
            }
            fwrite(&(p.r), 1, 1, file);
            fwrite(&(p.g), 1, 1, file);
            fwrite(&(p.b), 1, 1, file);
        }
    }
    fclose(file);
    return 1;
}

// --- Queue Functions (Full Implementation) ---
queue *queue_new() {
    queue *q = (queue *)malloc(sizeof(queue));
    if (!q) return NULL;
    q->head = q->tail = NULL;
    q->nb_elements = 0;
    return q;
}

void queue_free(queue *q) {
    if (!q) return;
    element *current = q->head;
    while (current) {
        element *next = current->next;
        free(current->content);
        free(current);
        current = next;
    }
    free(q);
}

int queue_send(queue *q, command *comm) {
    element *e = (element *)malloc(sizeof(element));
    if (!e) return 0;
    e->content = comm;
    e->next = NULL;
    if (queue_is_empty(q)) {
        q->head = q->tail = e;
    } else {
        q->tail->next = e;
        q->tail = e;
    }
    q->nb_elements++;
    return 1;
}

command *queue_receive(queue *q) {
    if (queue_is_empty(q)) return NULL;
    element *e = q->head;
    command *comm = e->content;
    q->head = e->next;
    if (!q->head) q->tail = NULL;
    free(e);
    q->nb_elements--;
    return comm;
}

int queue_is_empty(queue *q) {
    return q->nb_elements == 0;
}

unsigned queue_size(queue *q) {
    return q->nb_elements;
}

void ajouter_commande_normale(type_mouvement t, int d, float v, int p, queue *q) {
    command *c = (command*)malloc(sizeof(command));
    c->type = t;
    c->duree = (float)d;
    c->vitesse = v;
    c->priorite = p;
    queue_send(q, c);
}

void ajouter_commande_urgente(type_mouvement t, int d, float v, int p, queue *q) {
    ajouter_commande_normale(t, d, v, p, q);
}

const char* mouvementToString(type_mouvement t) {
    switch (t) {
        case AVANCER: return "AVANCER";
        case RECULER: return "RECULER";
        case TOURNER_GAUCHE: return "TOURNER_GAUCHE";
        case TOURNER_DROITE: return "TOURNER_DROITE";
        case ARRET_URGENCE: return "ARRET_URGENCE";
    }
    return "INCONNU";
}

void executerCommande(command *c, FILE *journal) {
    if (!c || !journal) return;
    // Récupérer l'heure actuelle
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char timestamp[16];
    strftime(timestamp, sizeof(timestamp), "%H:%M:%S", t);

    // --- Affichage à l'écran ---
    if (c->priorite == 1) { // URGENTE
        if (c->type == ARRET_URGENCE) {
            printf("COMMANDE URGENTE : %s\n", mouvementToString(c->type));
        } else {
            printf("COMMANDE URGENTE : %s %fs\n", mouvementToString(c->type), c->duree);
        }
    } else { // NORMALE
        printf("Commande normale : %s %fs\n", mouvementToString(c->type), c->duree);
    }

    // --- Sauvegarde dans le fichier ---
    if (c->priorite == 1) {
        if (c->type == ARRET_URGENCE) {
            fprintf(journal, "* [%s] URGENTE : %s (vitesse=%.2f) *\n",
                    timestamp, mouvementToString(c->type), c->vitesse);
        } else {
            fprintf(journal, "* [%s] URGENTE : %s (durée=%f, vitesse=%.2f) *\n",
                    timestamp, mouvementToString(c->type), c->duree, c->vitesse);
        }
    } else {
        fprintf(journal, "[%s] NORMALE : %s (durée=%f, vitesse=%.2f)\n",
                timestamp, mouvementToString(c->type), c->duree, c->vitesse);
    }
    fflush(journal);
}

int check_robot_collision(int center_y, int center_x, int **map) {
    if (center_y < 2 || center_y >= MAP_SIZE - 2 || center_x < 2 || center_x >= MAP_SIZE - 2) {
        return 0;
    }
    for (int y = center_y - 2; y <= center_y + 2; y++) {
        for (int x = center_x - 2; x <= center_x + 2; x++) {
            if (map[y][x] == 0 || map[y][x] == 2) {
                return 0;
            }
        }
    }
    return 1; // Safe
}

int check_front_sensor(int **map) {
    int cy = coordonne_robot[0];
    int cx = coordonne_robot[1];
    int dy = direction_robot[0];
    int dx = direction_robot[1];
    int perp_dy = -dx;
    int perp_dx = dy;
    for (int k = 1; k <= 3; k++) {
        int center_y = cy + k * dy;
        int center_x = cx + k * dx;
        for (int w = -2; w <= 2; w++) {
            int sy = center_y + w * perp_dy;
            int sx = center_x + w * perp_dx;

            if (!in_bounds(sy, sx)) return 1;

            if (map[sy][sx] == 0) {
                return 1;
            }
        }
    }
    return 0;
}

int evade_obstacle(int **map, int waypoints[MAP_SIZE*MAP_SIZE][2], int *current_wp_index, int n_waypoints, queue *n, queue *u, FILE *journal) {
    fprintf(journal, "!!! OBSTACLE DETECTED. INITIATING EVASION (Temporarily using Unauthorized Zone). !!!\n");

    ajouter_commande_urgente(ARRET_URGENCE, 0, 0, 1, u);
    prendre_prochaine_commande(u, n, journal);

    EVASION_MODE = 1;

    const int MAX_EVASION_STEPS = 100;
    int evasion_steps = 0;

    int cur_dir_idx = -1;
    if (direction_robot[0] == -1 && direction_robot[1] == 0) cur_dir_idx = 0;
    else if (direction_robot[0] == 0 && direction_robot[1] == 1) cur_dir_idx = 1;
    else if (direction_robot[0] == 1 && direction_robot[1] == 0) cur_dir_idx = 2;
    else if (direction_robot[0] == 0 && direction_robot[1] == -1) cur_dir_idx = 3;

    int desired_dir_idx = (cur_dir_idx + 1) % 4;
    rotate_to(cur_dir_idx, desired_dir_idx, n);
    prendre_prochaine_commande(u, n, journal);
    cur_dir_idx = desired_dir_idx;

    while (evasion_steps < MAX_EVASION_STEPS) {

        int step_result = step_forward(n);
        prendre_prochaine_commande(u, n, journal);

        int robot_center_cell = map[coordonne_robot[0]][coordonne_robot[1]];
        if (robot_center_cell == 1 || robot_center_cell == 3 || robot_center_cell == 4) {

            if (check_front_sensor(map) == 0) {
                EVASION_MODE = 0; // Exit evasion mode
                fprintf(journal, "Evasion successful. Rejoined main road at (%d, %d).\n", coordonne_robot[0], coordonne_robot[1]);

                if (*current_wp_index < n_waypoints - 2) {
                    *current_wp_index += 2;
                }
                return 1;
            }
        }

        if (check_front_sensor(map) == 1 || step_result == 0) {
            desired_dir_idx = (cur_dir_idx + 3) % 4; // Turn Left
            rotate_to(cur_dir_idx, desired_dir_idx, n);
            prendre_prochaine_commande(u, n, journal);
            cur_dir_idx = desired_dir_idx;
        }

        evasion_steps++;
    }

    EVASION_MODE = 0;
    fprintf(journal, "Evasion failed after %d steps. Robot is stuck.\n", MAX_EVASION_STEPS);
    return 0; // Failure
}

int move_robot(command *comm) {
    if (!comm) return -1;

    if (comm->type == AVANCER) {
        int d = (int) roundf(comm->duree * comm->vitesse) / 10;

        for (int step = 0; step < d; ++step) {
            int new_y = coordonne_robot[0] + direction_robot[0];
            int new_x = coordonne_robot[1] + direction_robot[1];

            int safe_move = 1;

            if (EVASION_MODE) {
                for (int y = new_y - 2; y <= new_y + 2; y++) {
                    for (int x = new_x - 2; x <= new_x + 2; x++) {
                        if (!in_bounds(y, x) || map[y][x] == 0) {
                            safe_move = 0;
                            break;
                        }
                    }
                    if (!safe_move) break;
                }
            } else {
                if (check_robot_collision(new_y, new_x, map) == 0) {
                    safe_move = 0;
                }
            }

            if (!safe_move) return -1;

            if (steps < MAP_SIZE * MAP_SIZE) {
                robot_path[steps][0] = new_y;
                robot_path[steps][1] = new_x;
                steps++;
            }
            coordonne_robot[0] = new_y;
            coordonne_robot[1] = new_x;
        }
    }
    if (comm->type == TOURNER_GAUCHE) {
        int dy = direction_robot[0], dx = direction_robot[1];
        direction_robot[0] = -dx;
        direction_robot[1] = dy;
    }
    if (comm->type == TOURNER_DROITE) {
        int dy = direction_robot[0], dx = direction_robot[1];
        direction_robot[0] = dx;
        direction_robot[1] = -dy;
    }
    return 0;
}

void prendre_prochaine_commande(queue* u, queue* n, FILE* j){
    command * comm;
    while (!queue_is_empty(u)) {
        comm = queue_receive(u);
        executerCommande(comm, j);
        move_robot(comm);
        free(comm);
    }
    while (!queue_is_empty(n)) {
        comm = queue_receive(n);
        executerCommande(comm, j);
        move_robot(comm);
        free(comm);
    }
}

void rotate_to(int cur, int desired_dir, queue *n) {
    int diff = (desired_dir - cur + 4) % 4;
    if (diff == 0) return;
    if (diff == 1) {
        ajouter_commande_normale(TOURNER_DROITE, 1, 0, 0, n);
    } else if (diff == 2) {
        ajouter_commande_normale(TOURNER_DROITE, 1, 0, 0, n);
        ajouter_commande_normale(TOURNER_DROITE, 1, 0, 0, n);
    } else if (diff == 3) {
        ajouter_commande_normale(TOURNER_GAUCHE, 1, 0, 0, n);
    }
}

int step_forward(queue* n) {
    ajouter_commande_normale(AVANCER, 1, (float) ROBOT_SPEED / 2, 0, n);
    return 1;
}

void pq_init(PriorityQueue* pq) { pq->size = 0; }

void pq_swap(Case** a, Case** b) { Case* temp = *a; *a = *b; *b = temp; }

void pq_bubble_up(PriorityQueue* pq, int index) {
    while (index > 0 && pq->items[index]->f < pq->items[(index - 1) / 2]->f) {
        pq_swap(&pq->items[index], &pq->items[(index - 1) / 2]);
        index = (index - 1) / 2;
    }
}

void pq_sink_down(PriorityQueue* pq, int index) {
    int min_index = index;
    int left = 2 * index + 1;
    int right = 2 * index + 2;

    if (left < pq->size && pq->items[left]->f < pq->items[min_index]->f) {
        min_index = left;
    }
    if (right < pq->size && pq->items[right]->f < pq->items[min_index]->f) {
        min_index = right;
    }

    if (min_index != index) {
        pq_swap(&pq->items[index], &pq->items[min_index]);
        pq_sink_down(pq, min_index);
    }
}

void pq_push(PriorityQueue* pq, Case* c) {
    if (pq->size < MAP_SIZE * MAP_SIZE) {
        pq->items[pq->size] = c;
        pq_bubble_up(pq, pq->size);
        pq->size++;
    }
}

Case* pq_pop_min(PriorityQueue* pq) {
    if (pq->size == 0) return NULL;
    Case* min_case = pq->items[0];
    pq->items[0] = pq->items[pq->size - 1];
    pq->size--;
    pq_sink_down(pq, 0);
    return min_case;
}

int pq_contains(PriorityQueue* pq, Case* c) {
    for (int i = 0; i < pq->size; i++) {
        if (pq->items[i] == c) return 1;
    }
    return 0;
}

int h_manhattan(Case* a, Case* b) {
    return abs(a->i - b->i) + abs(a->j - b->j);
}

void construireGrille(Case grille[MAP_SIZE][MAP_SIZE], int** map) {
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            grille[i][j].val = map[i][j];
            grille[i][j].g = INT_MAX;
            grille[i][j].h = 0;
            grille[i][j].f = INT_MAX;
            grille[i][j].visited = 0;
            grille[i][j].parent = NULL;
            grille[i][j].i = i;
            grille[i][j].j = j;
            for (int k = 0; k < 4; k++) grille[i][j].voisins[k] = NULL;
        }
    }

    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            int neighbors[4][2] = {{i-1, j}, {i+1, j}, {i, j-1}, {i, j+1}};
            for (int k = 0; k < 4; k++) {
                int ni = neighbors[k][0];
                int nj = neighbors[k][1];
                if (in_bounds(ni, nj)) {
                    grille[i][j].voisins[k] = &grille[ni][nj];
                }
            }
        }
    }
}

int trouverDepartArrivee(Case** start, Case** goal, Case grille[MAP_SIZE][MAP_SIZE]) {
    *start = NULL;
    *goal = NULL;
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            if (grille[i][j].val == 3) *start = &grille[i][j];
            if (grille[i][j].val == 4) *goal = &grille[i][j];
        }
    }
    return (*start != NULL && *goal != NULL);
}

int astar(Case* start, Case* goal) {
    PriorityQueue open_set;
    pq_init(&open_set);

    start->g = 0;
    start->h = h_manhattan(start, goal);
    start->f = start->g + start->h;
    pq_push(&open_set, start);

    while (open_set.size > 0) {
        Case* current = pq_pop_min(&open_set);
        current->visited = 1;

        if (current == goal) return 1;

        for (int k = 0; k < 4; k++) {
            Case* neighbor = current->voisins[k];
            if (neighbor && !neighbor->visited) {
                // Only allow movement on ROAD (1), START (3), or GOAL (4)
                if (neighbor->val == 0 || neighbor->val == 2) continue;

                int tentative_g = current->g + 1;

                if (tentative_g < neighbor->g) {
                    neighbor->parent = current;
                    neighbor->g = tentative_g;
                    neighbor->h = h_manhattan(neighbor, goal);
                    neighbor->f = neighbor->g + neighbor->h;

                    if (!pq_contains(&open_set, neighbor)) {
                        pq_push(&open_set, neighbor);
                    }
                }
            }
        }
    }
    return 0;
}

void marquerChemin(Case* start, Case* goal, int** map) {
    Case* current = goal->parent;
    while (current != start) {
        if (current->val == 1) {
            map[current->i][current->j] = 5;
        }
        current = current->parent;
    }
}

int remplir_Chemin(Case* start, Case* goal, int path[MAP_SIZE*MAP_SIZE][2]) {
    int count = 0;
    Case* current = goal;
    while (current != NULL) {
        count++;
        current = current->parent;
    }

    current = goal;
    int index = count - 1;
    while (current != NULL) {
        path[index][0] = current->i;
        path[index][1] = current->j;
        index--;
        current = current->parent;
    }
    return count;
}

int move_to_waypoint(int wp[2], queue *n, queue *u, FILE* f) {
    int cur = -1;
    if (direction_robot[0] == -1 && direction_robot[1] == 0) cur = 0;
    else if (direction_robot[0] == 0 && direction_robot[1] == 1) cur = 1;
    else if (direction_robot[0] == 1 && direction_robot[1] == 0) cur = 2;
    else if (direction_robot[0] == 0 && direction_robot[1] == -1) cur = 3;

    int watchdog = 0;
    const int MAX_WATCHDOG = 500;

    while (abs(coordonne_robot[1] - wp[1]) > 0) {
        if (++watchdog > MAX_WATCHDOG) return 0;
        int dx = wp[1] - coordonne_robot[1];
        int desired_dir = (dx > 0) ? 1 : 3;
        rotate_to(cur, desired_dir, n);
        prendre_prochaine_commande(u, n, f);
        cur = desired_dir;
        step_forward(n);
        prendre_prochaine_commande(u, n, f);
    }
    while (abs(coordonne_robot[0] - wp[0]) > 0) {
        if (++watchdog > MAX_WATCHDOG * 2) return 0;
        int dy = wp[0] - coordonne_robot[0];
        int desired_dir = (dy < 0) ? 0 : 2;
        rotate_to(cur, desired_dir, n);
        prendre_prochaine_commande(u, n, f);
        cur = desired_dir;
        step_forward(n);
        prendre_prochaine_commande(u, n, f);
    }

    while (abs(coordonne_robot[1] - wp[1]) > 0) {
        if (++watchdog > MAX_WATCHDOG * 3) return 0;
        int dx = wp[1] - coordonne_robot[1];
        int desired_dir = (dx > 0) ? 1 : 3;
        rotate_to(cur, desired_dir, n);
        prendre_prochaine_commande(u, n, f);
        cur = desired_dir;
        step_forward(n);
        prendre_prochaine_commande(u, n, f);
    }

    return 1;
}

