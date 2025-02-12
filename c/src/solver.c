#include "solver.h"
#include <stdio.h>
#include <stdlib.h>

void init_solver(Solver* solver, Model* model, float* state, float t0, float tf, float dt, DisturbanceFunction disturbance_function, NumericalSolverFunction solve_step_function) {
    solver->model = model;
    solver->state = state;
    solver->t0 = t0;
    solver->tf = tf;
    solver->dt = dt;
    solver->disturbance_function = disturbance_function;
    solver->solve_step_function = solve_step_function;
};

float* calculate_state_dot(Model* model, float* state, float* disturbances) {

    float* state_dot = (float*)malloc(sizeof(float) * NUM_STATES);

    // Calculate state_dot for each state
    for (size_t i = 0; i < NUM_STATES; i++){
        float num = 0;

        // Propagate plant
        for (size_t j = 0; j < NUM_STATES; j++) {
            num += state[j] * model->plant[i][j];
        }

        // Propagate disturbances
        for (size_t k = 0; k < NUM_DISTURBANCE_STATES; k++) {
            num += disturbances[k] * model->disturbance_plant[i][k];
        }
        
        // Update state_dot array
        state_dot[i] = num;
    }

    return state_dot;
}

// Solves w/ forward integration for a single step
void forward_integration(float* state, float* state_dot, float dt) {
    for (size_t i = 0; i < NUM_STATES; i++) {
        state[i] += state_dot[i] * dt;
    }
}

void run_forward_integration_solver(Solver* solver, FILE* file) {

    // Write header
    fprintf(file, "time");
    for (size_t i = 0; i < NUM_STATES; i++) {
        fprintf(file, ",%s", solver->model->stateNames[i]);
    }
    for (size_t i = 0; i < NUM_DISTURBANCE_STATES; i++) {
        fprintf(file, ",%s", solver->model->disturbanceNames[i]);
    }
    fprintf(file, "\n");

    // Write initial state
    float t = solver->t0;
    while (t <= solver->tf) {
        printf("t = %f tf = %f dt = %f\n", t, solver->tf, solver->dt);

        // Calculate disturbance
        float* disturbances = solver->disturbance_function(t);

        // Calculate state dot
        float* state_dot = calculate_state_dot(solver->model, solver->state, disturbances);

        // Run one step of forward integration
        solver->solve_step_function(solver->state, state_dot, solver->dt);
        free(state_dot);
        

        // Increment time
        t += solver->dt;

        // Log time 
        fprintf(file, "%f", t);
        // Log states
        for (size_t i = 0; i < NUM_STATES; i++) {
            fprintf(file, ",%f", solver->state[i]);
        }
        // Log disturbances
        for (size_t i = 0; i < NUM_DISTURBANCE_STATES; i++) {
            fprintf(file, ",%f", disturbances[i]);
        }
        free(disturbances);
        fprintf(file, "\n");


    }
};