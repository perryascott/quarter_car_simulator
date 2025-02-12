#include "model.h"
#include "solver.h"
#include "disturbance.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char* argv[]) {

    if (argc < 2) {
        printf("Usage: %s <output_file>\n", argv[0]);
        return 1;
    }
    
    const char* output_file = argv[1];

    // Define system variables
    float ks = 25000; // N / m
    float ku = 200000; // N / m
    float bs = 1500; // Ns / m
    float bu = 0; // Ns / m
    float ms = 300; // kg
    float mu = 50; //kg

    // Define plant, and disturbances
    Model model = {
        {
            {0, 0, 1, 0},
            {0, 0, 0, 1},
            {-ks / ms, ks / ms, -bs / ms, bs / ms},
            {ks / mu, -(ku + ks) / mu, bs / mu, -(bs + bu) / mu}
        },
        {"x1", "x2", "x1_dot", "x2_dot"},
        {
            {0, 0},
            {0, 0},
            {0, 0},
            {ku / mu, bu / mu}
        },
        {"xg", "xg_dot"}
    };

    // Define initial conditions
    float state[NUM_STATES] = {0.0, 0.125, 0.0, 0.0};

    // Define simulation times (seconds)
    float dt = 0.0001;
    float t0 = 0;
    float tf = 20;

    // Define state names
    const char* stateNames[NUM_STATES] = {"x1", "x2", "x1_dot", "x2_dot"};

    // Solvers
    Solver solver;

    // Select disturbance and solver type
    // init_solver(&solver, &model, state, t0, tf, dt, no_disturbance, forward_integration);
    // init_solver(&solver, &model, state, t0, tf, dt, sinusoidal_disturbance, forward_integration);
    init_solver(&solver, &model, state, t0, tf, dt, trapezoidal_wave_disturbance, forward_integration);

    // Open file for logging
    FILE* file = fopen(output_file, "w");
    if (!file) {
        perror("Error opening file");
        return 1;
    }

    // Run simulation
    run_forward_integration_solver(&solver, file);

    fclose(file);
    return 0;
}