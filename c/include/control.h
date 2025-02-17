#ifndef CONTROL_H
#define CONTROL_H
#include "model.h"
#include <stdint.h>

// Solver function typedef
typedef float* (*ControlLawFunction)(Model*, float*, float*, void*);

// Generic Controller Struct
typedef struct {
    ControlLawFunction control_law;  // Function pointer for the control law
    void* controller_data;           // Pointer to controller-specific data
    void (*free_controller_data)(void*); // Function pointer to free data
} Controller;

Controller* create_pid_controller(float p, float i, float d, float dt);
Controller* create_non_controller();

// PID Controller Struct
typedef struct {
    float p, i, d;
    float dt;
    float error_prev;
    float error_integral;
    float* disturbance;
    float* prev_state;
    float* prev_prev_state;
    uint8_t states_saved;
} PIDControllerData;


#endif