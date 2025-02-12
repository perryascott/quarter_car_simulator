#ifndef SOLVER_H
#define SOLVER_H
#include "model.h"
#include <stdio.h>

// Disturbance function typedef
typedef float* (*DisturbanceFunction)(float);

// Solver function typedef
typedef void (*NumericalSolverFunction)(float* state, float* state_dot, float dt);

typedef struct {

    // Model
    Model* model;

    // States
    float* state;

    // Disturbance function
    DisturbanceFunction disturbance_function;

    // Solver function
    NumericalSolverFunction solve_step_function;

    // Simulation time parameters
    float t0, tf, dt;

} Solver;

void init_solver(Solver* solver, Model* model, float* state, float t0, float tf, float dt, DisturbanceFunction disturbance_function, NumericalSolverFunction solve_step_function);
void forward_integration(float* state, float* state_dot, float dt);
void run_forward_integration_solver(Solver* solver, FILE* file);

#endif