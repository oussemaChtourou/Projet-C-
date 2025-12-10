#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "map.h"

#define MAP_SIZE 100

#ifdef _WIN32
#include <windows.h>
#define sleep_ms(ms) Sleep(ms)
#else
#include <unistd.h>
#define sleep_ms(ms) usleep((ms) * 1000)
#endif

int main() {
    map = create_map();
    if (!map) {
        printf("Failed to create map\n");
        return 1;
    }
    create_image("map.ppm", map);
    printf("map.ppm created\n");

    int** map1 = malloc(MAP_SIZE * sizeof(int*));
    if (!map1) { perror("malloc map1"); return 1; }
    for (int y = 0; y < MAP_SIZE; y++) {
        map1[y] = malloc(MAP_SIZE * sizeof(int));
        memcpy(map1[y], map[y], MAP_SIZE * sizeof(int));
    }

    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            if (map[y][x] == 0) { // check original map for obstacle
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        int ny = y + dy;
                        int nx = x + dx;
                        // Mark road (1) as unauthorized zone (2) if it's not start/goal
                        if (in_bounds(ny, nx) && map1[ny][nx] == 1 &&
                            !(ny == 95 && nx == 4) && !(ny == 4 && nx == 95)) {
                            map1[ny][nx] = 0;
                        }
                    }
                }
            }
        }
    }

    Case grille[MAP_SIZE][MAP_SIZE];
    construireGrille(grille, map1);
    Case* start = NULL;
    Case* goal  = NULL;
    if (!trouverDepartArrivee(&start, &goal, grille)) {
        printf("Start or Goal not found.\n");
        return 1;
    }

    int shortest_robot_path[MAP_SIZE*MAP_SIZE][2];
    int l_path = 0;
    int trouve = astar(start, goal);

    if (trouve) {
        l_path = remplir_Chemin(start, goal, shortest_robot_path);
        marquerChemin(start, goal, map1);
        create_image("map1.ppm", map1);
        printf("map1.ppm created (A* Path)\n");
    } else {
        printf("No path found.\n");
        for (int y = 0; y < MAP_SIZE; y++) free(map1[y]);
        free(map1);
        return 1;
    }

    FILE * journal = fopen("journal_robot.txt", "wt");
    if (!journal) { perror("Error opening journal_robot.txt"); return 1; }

    queue *normal_command_queue = queue_new();
    queue *urgent_command_queue = queue_new();
    if (!normal_command_queue || !urgent_command_queue) {
        printf("Failed to create queues\n");
        fclose(journal);
        return 1;
    }

    int current_wp_index = 0;

    printf("Starting robot navigation...\n");
    while (current_wp_index < l_path) {
        if (check_front_sensor(map)) {
            int success = evade_obstacle(map, shortest_robot_path, &current_wp_index, l_path, normal_command_queue, urgent_command_queue, journal);
            if (!success) {
                printf("NAVIGATION FAILED: Robot stuck during evasion.\n");
                break;
            }
            continue;
        }

        if (abs(coordonne_robot[0] - shortest_robot_path[current_wp_index][0]) <= 1 &&
            abs(coordonne_robot[1] - shortest_robot_path[current_wp_index][1]) <= 1) {
            current_wp_index++;
            continue;
        }

        int next_wp[2];
        next_wp[0] = shortest_robot_path[current_wp_index][0];
        next_wp[1] = shortest_robot_path[current_wp_index][1];

        if (move_to_waypoint(next_wp, normal_command_queue, urgent_command_queue, journal) == 0) {
            fprintf(journal, "NAVIGATION FAILED: Could not reach waypoint %d.\n", current_wp_index);
            printf("NAVIGATION FAILED: Could not reach waypoint %d.\n", current_wp_index);
            break;
        }

        prendre_prochaine_commande(urgent_command_queue, normal_command_queue, journal);
    }

    printf("Navigation finished. Final Path Length: %d steps.\n", steps);
    int** map2 = malloc(MAP_SIZE * sizeof(int*));
    if (!map2) {
        perror("malloc map2");
    }
    else {
        for (int y = 0; y < MAP_SIZE; y++) {
            map2[y] = malloc(MAP_SIZE * sizeof(int));
            if (!map2[y]) {
                perror("malloc map2 row");
                for (int k = 0; k < y; k++) free(map2[k]);
                free(map2);
                map2 = NULL;
                break;
            }
            for (int x = 0; x < MAP_SIZE; x++) map2[y][x] = map[y][x];
        }
    }
    for (int i = 0; i < steps && map2 != NULL; i++) {
        int cy = robot_path[i][0];
        int cx = robot_path[i][1];
        for (int y = cy - 2; y <= cy + 2; y++) {
            for (int x = cx - 2; x <= cx + 2; x++) {
                if (y >= 0 && y < MAP_SIZE && x >= 0 && x < MAP_SIZE) {
                    map2[y][x] = 6;
                }
            }
        }
    }
    if (map2) {
        create_image("map2.ppm", map2);
        printf("map2.ppm created\n");
        for (int y = 0; y < MAP_SIZE; y++) free(map2[y]);
        free(map2);
    }

    fclose(journal);
    queue_free(normal_command_queue);
    queue_free(urgent_command_queue);
    for (int y = 0; y < MAP_SIZE; y++) free(map1[y]);
    free(map1);
    return 0;
}