#include "elevator.h"

#include <assert.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static struct passenger
{
    int from_floor;
    int to_floor;
} Passenger[PASSENGERS];

int passenger_ready_for_pickup[PASSENGERS];
int passenger_in_elevator[PASSENGERS];
int passenger_exited_elevator[PASSENGERS];

int elevator_at_pickup[PASSENGERS];
int elevator_at_destination[PASSENGERS];

pthread_mutex_t lock;

pthread_cond_t passenger_rdy[PASSENGERS];
pthread_cond_t elev_rdy_pickup[PASSENGERS];
pthread_cond_t elev_at_dest[PASSENGERS];
pthread_cond_t passenger_in[PASSENGERS];
pthread_cond_t passenger_out[PASSENGERS];
pthread_cond_t req_rdy;

int num_reqs = 0;
int passenger_elevator[PASSENGERS];

void initializer(void)
{
    // initialize some data structures
    for (int i = 0; i < PASSENGERS; i++)
    {
        passenger_elevator[i] = -1;
    }

    for (int i = 0; i < PASSENGERS; i++)
    {
        // memset(&Passenger[i], 0x00, sizeof(Passenger[i]));

        passenger_ready_for_pickup[i] = 0;
        passenger_in_elevator[i] = 0;
        passenger_exited_elevator[i] = 0;

        elevator_at_pickup[i] = 0;
        elevator_at_destination[i] = 0;

        pthread_cond_init(&passenger_rdy[i], NULL);
        pthread_cond_init(&elev_rdy_pickup[i], NULL);
        pthread_cond_init(&elev_at_dest[i], NULL);
        pthread_cond_init(&passenger_in[i], NULL);
        pthread_cond_init(&passenger_out[i], NULL);
    }

    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&req_rdy, NULL);

    num_reqs = PASSENGERS * TRIPS_PER_PASSENGER;
}

void passenger_controller(int passenger, int from_floor, int to_floor,
                          void (*enter_elevator)(int, int), void (*exit_elevator)(int, int))
{
    pthread_mutex_lock(&lock);

    elevator_at_pickup[passenger] = 0;
    passenger_in_elevator[passenger] = 0;
    elevator_at_destination[passenger] = 0;
    passenger_exited_elevator[passenger] = 0;
    passenger_elevator[passenger] = -1;

    // inform elevator of floor
    Passenger[passenger].from_floor = from_floor;
    Passenger[passenger].to_floor = to_floor;

    // signal ready and wait
    passenger_ready_for_pickup[passenger] = 1;
    pthread_cond_signal(&req_rdy);

    while (!elevator_at_pickup[passenger])
    {
        pthread_cond_wait(&elev_rdy_pickup[passenger], &lock);
    }
    int elev = passenger_elevator[passenger];
    pthread_mutex_unlock(&lock);

    // enter elevator and wait
    enter_elevator(passenger, elev);

    pthread_mutex_lock(&lock);
    passenger_in_elevator[passenger] = 1;
    pthread_cond_signal(&passenger_in[passenger]);

    while (!elevator_at_destination[passenger])
    {
        pthread_cond_wait(&elev_at_dest[passenger], &lock);
    }
    pthread_mutex_unlock(&lock);

    // exit elevator and signal
    exit_elevator(passenger, elev);

    pthread_mutex_lock(&lock);

    passenger_elevator[passenger] = -1;
    passenger_exited_elevator[passenger] = 1;
    num_reqs--;

    pthread_cond_signal(&passenger_out[passenger]);

    if (num_reqs == 0)
        pthread_cond_broadcast(&req_rdy); // wake all waiting threads up

    pthread_mutex_unlock(&lock);
}

// example procedure that makes it easier to work with the API
// move elevator from source floor to destination floor
// you will probably have to modify this in the future ...

static void move2dest(int source, int destination, void (*move_elevator)(int, int), int elevator)
{
    int direction, steps;
    int distance = destination - source;
    if (distance > 0)
    {
        direction = 1;
        steps = distance;
    }
    else
    {
        direction = -1;
        steps = -1 * distance;
    }
    for (; steps > 0; steps--)
        move_elevator(elevator, direction);
}

void elevator_controller(int elevator, int at_floor,
                         void (*move_elevator)(int, int),
                         void (*open_door)(int),
                         void (*close_door)(int))
{
    int elev_curr_floor = at_floor;

    while (1)
    {

        pthread_mutex_lock(&lock);

        if (num_reqs == 0)
        {
            pthread_mutex_unlock(&lock);
            return;
        }

        int passenger = -1;
        int worst_case = 2 * FLOORS;

        for (int i = 0; i < PASSENGERS; i++)
        {

            if (passenger_ready_for_pickup[i] && passenger_elevator[i] == -1)
            {
                int distance = abs(Passenger[i].from_floor - elev_curr_floor);

                if (distance < worst_case)
                {
                    worst_case = distance;
                    passenger = i;
                }
            }
        }

        if (passenger == -1)
        {
            pthread_cond_wait(&req_rdy, &lock);
            pthread_mutex_unlock(&lock);
            continue;
        }

        passenger_elevator[passenger] = elevator;
        passenger_ready_for_pickup[passenger] = 0;

        elevator_at_pickup[passenger] = 0;
        passenger_in_elevator[passenger] = 0;
        elevator_at_destination[passenger] = 0;
        passenger_exited_elevator[passenger] = 0;

        int from = Passenger[passenger].from_floor;
        int to = Passenger[passenger].to_floor;
        pthread_mutex_unlock(&lock);

        // wait for passenger to press button and move
        move2dest(elev_curr_floor, from, move_elevator, elevator);
        elev_curr_floor = from;

        // open door and signal passenger
        open_door(elevator);

        pthread_mutex_lock(&lock);
        elevator_at_pickup[passenger] = 1;
        pthread_cond_signal(&elev_rdy_pickup[passenger]);

        // wait for passenger to enter then close and move
        while (!passenger_in_elevator[passenger])
            pthread_cond_wait(&passenger_in[passenger], &lock);
        pthread_mutex_unlock(&lock);

        close_door(elevator);

        move2dest(elev_curr_floor, to, move_elevator, elevator);
        elev_curr_floor = to;

        // open door signal
        open_door(elevator);

        pthread_mutex_lock(&lock);
        elevator_at_destination[passenger] = 1;
        pthread_cond_signal(&elev_at_dest[passenger]);

        // wait for passenger to leave and close door
        while (!passenger_exited_elevator[passenger])
            pthread_cond_wait(&passenger_out[passenger], &lock);
        pthread_mutex_unlock(&lock);

        close_door(elevator);
    }
}
