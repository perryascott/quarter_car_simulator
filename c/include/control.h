#ifndef CONTROL_H
#define CONTROL_H
#include "model.h"

// Solver function typedef
typedef float* (*ControlLawFunction)(Model*, float*, void*);

// Generic Controller Struct
typedef struct {
    ControlLawFunction control_law;  // Function pointer for the control law
    void* controller_data;           // Pointer to controller-specific data
    void (*free_controller_data)(void*); // Function pointer to free data
} Controller;

Controller* create_pid_controller(float p, float i, float d, float dt);

// PID Controller Struct
typedef struct {
    float p, i, d;
    float dt;
    float error_prev;
    float error_integral;
    float* prev_state;
} PIDControllerData;


#endif