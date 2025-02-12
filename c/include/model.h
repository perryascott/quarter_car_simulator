#ifndef MODEL_H
#define MODEL_H
#include <stddef.h>

static const size_t NUM_STATES = 4;
static const size_t NUM_DISTURBANCE_STATES = 2;
static const size_t NUM_CONTROL_INPUTS = 1;

typedef struct {

    // Plant
    float plant[NUM_STATES][NUM_STATES];
    char* stateNames[NUM_STATES];

    // Control
    float control_plant[NUM_STATES][NUM_CONTROL_INPUTS];
    char* controlNames[NUM_CONTROL_INPUTS];

    // Disturbance
    float disturbance_plant[NUM_STATES][NUM_DISTURBANCE_STATES];
    char* disturbanceNames[NUM_DISTURBANCE_STATES];

} Model;

#endif