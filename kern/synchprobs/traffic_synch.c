#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>

/*
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/*
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to
 * declare other global variables if your solution requires them.
 */


/*
 * This is the definition of Direction
 * enum Directions
 * {
 *    north = 0,
 *    east = 1,
 *    south = 2,
 *    west = 3
 *  };
 *  typedef enum Directions Direction;
 */

/*
 * This is the declarations of Vehicles in "traffic.c".
 * To make variable consistency, we still use this variable name
 * typedef struct Vehicles
 * {
 *    Direction origin;
 *    Direction destination;
 *  } Vehicle;
 */

// declarations of vehicles in "traffic_synch.c"
typedef struct vehicles {
    Direction origin;
    Direction destination;
} vehicle;

// declarations of lock for "traffic intersection"
static struct lock *intersectionLock;

// declarations of condition variable for "traffic intersection"
static struct cv *intersectionCv;

/* helper functions to check conditions
 * "intersection_turn_right" checks whether the vehicle will turn right at the intersection
 * "intersection_check" checks whether the vehicle satisfies those three conditions
 * "enter_intersection" checks whether the vehicle could enter the intersection
 * "exit_intersection" helps to remove the leaving vehicle and awake the waiting vehicles
 *
 */
static bool intersection_turn_right(vehicle *v);
static int intersection_check(vehicle *V_a, vehicle *V_b);
static bool enter_intersection(Direction origin, Direction destination);

/*  vehicles in the intersection */
/*  The vehicle simulation works by creating a fixed number of concurrent threads
 *  Each tread simulates a sequence of vehicles attempting to pass through the intersection
 */
struct array *intersectionVehicle;

/*
 * array_create, array_init, array_destroy, array_cleanup, array_setsize, array_remove
 * are in the "array.c" file.
 * array_num, array_get, array_add are in the "array.h" file.
 * need to use these functions to help deal with vehicles in the intersection
 * vehicles not in the intersection will wait (cv_wait to lock the cv and lock) and sleep
 */



/*
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 *
 */
void
intersection_sync_init(void)
{

    // create lock and condition varaible for the traffic
    intersectionLock = lock_create("intersectionLock");
    intersectionCv = cv_create("intersectionCv");
    intersectionVehicle = array_create();
    array_init(intersectionVehicle);

    if ((intersectionLock == NULL) || (intersectionCv == NULL) || (intersectionVehicle == NULL)) {
        panic("intersection sync initialization failed");
    }

    return;
}

/*
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{

    // when intersection syn cleanup, we need to check whether we have those primitives
    // such as locks, condition variables.
    // Besides, if the the array is empty, we don't need to cleanup again
    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCv != NULL);
    KASSERT(intersectionVehicle != NULL);

    lock_destroy(intersectionLock);
    cv_destroy(intersectionCv);
    array_destroy(intersectionVehicle);

    return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

bool
intersection_turn_right(vehicle *v)
{
    if (((v->origin==0) && (v->destination == 3)) ||
        ((v->origin==1) && (v->destination == 0)) ||
        ((v->origin==2) && (v->destination == 1)) ||
        ((v->origin==3) && (v->destination == 2)))
    {
        return true;
    }
    return false;
}

int
intersection_check(vehicle *V_a, vehicle *V_b)
{
    if ((V_a->origin == V_b->origin) ||
       ((V_a->origin == V_b->destination) && (V_a->destination == V_b->origin)) ||
       ((V_a->destination != V_b->destination) && ((intersection_turn_right(V_a)) || (intersection_turn_right(V_b)))))
    {
      return 1;
    }
    return 0;
}

bool
enter_intersection(Direction origin, Direction destination)
{
    vehicle *v = kmalloc(sizeof(vehicle));
    v->origin = origin;
    v->destination = destination;
    for (unsigned int i = 0 ; i < array_num(intersectionVehicle); i++)
    {

          // check whether the vehicle v can enter the intersection or not
          // i.e. whether there is a conflict between v and vehicles in the intersectionVehicle
          if (intersection_check(v, array_get(intersectionVehicle, i)) != 1)
          {

              // if the vehicle can not enter the intersection, let it wait in the wait queue
              cv_wait(intersectionCv, intersectionLock);
              return false;
          }
    }

    // otherwise, the vehicle will enter the intersection
    // but we still need to check whether the thread holds the lock or not
    KASSERT(lock_do_i_hold(intersectionLock));
    array_add(intersectionVehicle, v, NULL);
    return true;
}


void
intersection_before_entry(Direction origin, Direction destination)
{

    // before entry, locks, cvs and arrays of vehicles should work fine.
    // otherwise, report the error
    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCv != NULL);
    KASSERT(intersectionVehicle != NULL);

    lock_acquire(intersectionLock);

    while (!enter_intersection(origin, destination)) {};

    lock_release(intersectionLock);

    return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination)
{
    // before entry, locks, cvs and arrays of vehicles should work fine.
    // otherwise, report the error
    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCv != NULL);
    KASSERT(intersectionVehicle != NULL);

    lock_acquire(intersectionLock);

    for (unsigned int i = 0 ; i  < array_num(intersectionVehicle); i++)
    {
        vehicle *current_vehicle = array_get(intersectionVehicle, i);

        // determine the it is the leaving vehicle
        if ((current_vehicle->origin == origin) && (current_vehicle->destination == destination))
        {

            // awake all the waiting vehicles and remove this vehicle from the intersection
            array_remove(intersectionVehicle, i);
            cv_broadcast(intersectionCv, intersectionLock);
            break;
        }
    }

    lock_release(intersectionLock);

    return;
}
