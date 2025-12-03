// tc.c  (Traffic Control System)
// Implements a multi-threaded four-way intersection using:
//   - per-direction head-of-line semaphores (fairness for each stop sign)
//   - same-direction "flow" similar to readers in readers–writers
//   - quadrant-level mutex locks to prevent collisions

// Each car is represented by a thread that:
//   1) Arrives at the intersection (ArriveIntersection)
//   2) Crosses the intersection (CrossIntersection)
//   3) Exits the intersection (ExitIntersection)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#include "common.h"
#include "common_threads.h"

#define NUM_CARS 8

// Directions: original and target
typedef struct _directions {
    char dir_original; // starting direction (N, S, W, E)
    char dir_target;   // target direction after intersection
} directions;

// Car information
typedef struct {
    int cid;             // car id (1..NUM_CARS)
    double arrival_time; // when it arrives at intersection (seconds)
    directions dirs;     // original + target directions
    int index;           // global arrival order (0..NUM_CARS-1)
} car_t;

// Direction indices for arrays
enum {DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3};

// Hard-coded cars (input example)
car_t cars[NUM_CARS] = {
    {1, 1.1, {'^', '^'}, 0},
    {2, 2.2, {'^', '^'}, 1},
    {3, 3.3, {'^', '<'}, 2},
    {4, 4.4, {'v', 'v'}, 3},
    {5, 5.5, {'v', '>'}, 4},
    {6, 6.6, {'^', '^'}, 5},
    {7, 7.7, {'>', '^'}, 6},
    {8, 8.8, {'<', '^'}, 7}
};

// Quadrant locks
pthread_mutex_t quad[4];    // quad 0 = NW, 1 = NE, 2 = SE, 3 = SW (counter-clockwise ordering)
pthread_mutex_t print_lock; // Mutex to serialize printing (so log lines don't interleave)

// Direction flow control
pthread_mutex_t turn_lock;   // protects current_direction & flow_count
pthread_cond_t  turn_cv;    //  used to wake waiting directions when flow ends

int current_direction = -1;  // -1 means no direction currently owns intersection
int flow_count[4] = {0};     // number of cars from each direction crossing

// Per-direction head-of-line semaphores
sem_t hol_sem[4];

// Simulation timing
double start_time;

// Helper functions ------------------------------------------------------------

// Map ASCII direction to internal enum index.
int dir_index(char d) {
    switch (d) {
        case '^': return DIR_N;
        case '>': return DIR_E;
        case 'v': return DIR_S;
        case '<': return DIR_W;
        default:  return -1;
    }
}

// Return time elapsed since program start.
double now() {
    return GetTime() - start_time;
}

// Thread-safe printing of events.
void log_event(const char *event, car_t *c) {
    Pthread_mutex_lock(&print_lock);
    printf("Time %.1f: Car %d (%c %c) %s\n",
           now(),
           c->cid,
           c->dirs.dir_original,
           c->dirs.dir_target,
           event);
    fflush(stdout);
    Pthread_mutex_unlock(&print_lock);
}

// Turn Interpretation (Left / Straight / Right) ---------------------------------

// Turn types
typedef enum {LEFT_T, STRAIGHT_T, RIGHT_T} turn_t;

// Determine type of turn based on original and target direction.
turn_t get_turn(directions *d) {
    char o = d->dir_original;
    char t = d->dir_target;

    if (o == t) return STRAIGHT_T;

    if (o == '^' && t == '>') return RIGHT_T;
    if (o == '^' && t == '<') return LEFT_T;

    if (o == 'v' && t == '<') return RIGHT_T;
    if (o == 'v' && t == '>') return LEFT_T;

    if (o == '>' && t == 'v') return RIGHT_T;
    if (o == '>' && t == '^') return LEFT_T;

    if (o == '<' && t == '^') return RIGHT_T;
    if (o == '<' && t == 'v') return LEFT_T;

    return STRAIGHT_T;
}

// Quadrant Mapping ------------------------------------------------------------

// Fill q[] with quadrants this car needs, return count
int get_quads(car_t *c, int q[]) {
    char o = c->dirs.dir_original;
    turn_t t = get_turn(&c->dirs);
    int n = 0;

    // quad 0 = NW, 1 = NE, 2 = SE, 3 = SW
    if (o == '^') {            // from North
        if (t == RIGHT_T) {    // N -> E
            q[n++] = 1;
        } else if (t == STRAIGHT_T) { // N -> S
            q[n++] = 1;
            q[n++] = 2;
        } else {               // N -> W (left)
            q[n++] = 1;
            q[n++] = 2;
            q[n++] = 3;
        }
    } else if (o == '>') {     // from East
        if (t == RIGHT_T) {    // E -> S
            q[n++] = 2;
        } else if (t == STRAIGHT_T) { // E -> W
            q[n++] = 2;
            q[n++] = 3;
        } else {               // E -> N (left)
            q[n++] = 2;
            q[n++] = 3;
            q[n++] = 0;
        }
    } else if (o == 'v') {     // from South
        if (t == RIGHT_T) {    // S -> W
            q[n++] = 3;
        } else if (t == STRAIGHT_T) { // S -> N
            q[n++] = 3;
            q[n++] = 0;
        } else {               // S -> E (left)
            q[n++] = 3;
            q[n++] = 0;
            q[n++] = 1;
        }
    } else if (o == '<') {     // from West
        if (t == RIGHT_T) {    // W -> N
            q[n++] = 0;
        } else if (t == STRAIGHT_T) { // W -> E
            q[n++] = 0;
            q[n++] = 1;
        } else {               // W -> S (left)
            q[n++] = 0;
            q[n++] = 1;
            q[n++] = 2;
        }
    }

    return n;
}

// Lock quadrants in sorted order to avoid circular wait.
void lock_quads(int q[], int n) {
    // sort q in ascending order, then lock
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (q[j] < q[i]) {
                int tmp = q[i];
                q[i] = q[j];
                q[j] = tmp;
            }
        }
    }
    for (int i = 0; i < n; i++) {
        Pthread_mutex_lock(&quad[q[i]]);
    }
}

// Unlock quadrants in reverse order.
void unlock_quads(int q[], int n) {
    for (int i = n - 1; i >= 0; i--) {
        Pthread_mutex_unlock(&quad[q[i]]);
    }
}

// Car actions ------------------------------------------------------------------

void ArriveIntersection(car_t *c);
void CrossIntersection(car_t *c);
void ExitIntersection(car_t *c);

// ARRIVE: head-of-line + direction/flow logic
void ArriveIntersection(car_t *c) {
    int d = dir_index(c->dirs.dir_original);

    log_event("arriving", c);

    // stop at stop sign for 2 seconds
    usleep(2000000);

    // Wait until this car is head-of-line for its direction
    sem_wait(&hol_sem[d]);

    // Now coordinate with other directions
    Pthread_mutex_lock(&turn_lock);

    // If another direction currently owns the intersection, wait.
    while (current_direction != -1 && current_direction != d) {
        pthread_cond_wait(&turn_cv, &turn_lock);
    }

    // At this point either intersection is free (-1) or already serving our direction
    if (current_direction == -1) {
        current_direction = d;   // take ownership
        flow_count[d] = 1;
    } else { // current_direction == d
        flow_count[d]++;         // join existing flow for our direction
    }

    Pthread_mutex_unlock(&turn_lock);
}

// CROSS: lock quadrants, simulate crossing, unlock quadrants
void CrossIntersection(car_t *c) {
    int q[4];
    int n = get_quads(c, q);

    lock_quads(q, n);

    log_event("crossing", c);

    // Simulate crossing time depending on turn type:
    // left:  5 seconds
    // straight: 4 seconds
    // right: 3 seconds
    turn_t t = get_turn(&c->dirs);
    if (t == LEFT_T) {
        Spin(5);
    } else if (t == STRAIGHT_T) {
        Spin(4);
    } else {
        Spin(3);
    }

    unlock_quads(q, n);
}

// EXIT: log exit and update flow control
void ExitIntersection(car_t *c) {
    int d = dir_index(c->dirs.dir_original);

    log_event("exiting", c);

    Pthread_mutex_lock(&turn_lock);

    flow_count[d]--;

    // if no more cars from this direction in intersection,
    // release ownership and wake all waiting directions
    if (flow_count[d] == 0) {
        current_direction = -1; // intersection becomes free
        pthread_cond_broadcast(&turn_cv); // wake all waiting directions
    }

    Pthread_mutex_unlock(&turn_lock);

    // Release head-of-line lock so next car from this direction
    // can move up to the stop sign
    sem_post(&hol_sem[d]);
}

// Car thread ------------------------------------------------------------
// The thread sleeps until arrival time,
// then performs ARRIVE → CROSS → EXIT in order.

void *Car(void *arg) {
    car_t *c = (car_t *)arg;

    // sleep until its arrival time
    unsigned int us = (unsigned int)(c->arrival_time * 1000000.0);
    usleep(us);

    // Perform the three actions in order
    ArriveIntersection(c);
    CrossIntersection(c);
    ExitIntersection(c);

    return NULL;
}


// main -------------------------------------------------------------------

int main() {

    /// Mark simulation start time
    start_time = GetTime();

    // Initialize print mutex
    Pthread_mutex_init(&print_lock, NULL);
    for (int i = 0; i < 4; i++) {
        Pthread_mutex_init(&quad[i], NULL);
    }
    
    // Initialize turn control mutex and condition variable
    Pthread_mutex_init(&turn_lock, NULL);
    pthread_cond_init(&turn_cv, NULL);

    // Initialize per-direction head-of-line semaphores
    for (int i = 0; i < 4; i++) {
        sem_init(&hol_sem[i], 0, 1);  // head-of-line token per direction
    }

    // Create car threads
    pthread_t tids[NUM_CARS];

    // Create one thread for each car
    for (int i = 0; i < NUM_CARS; i++) {
        Pthread_create(&tids[i], NULL, Car, &cars[i]);
    }

    // Wait for all cars to finish
    for (int i = 0; i < NUM_CARS; i++) {
        Pthread_join(tids[i], NULL);
    }

    return 0;
}
