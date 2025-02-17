#include "model.h"
#include "solver.h"
#include "disturbance.h"
#include "control.h"
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

    // Define system, control, and disturbance plants
    Model model = {

        // System plant
        {
            {0, 0, 1, 0},
            {0, 0, 0, 1},
            {-ks / ms, ks / ms, -bs / ms, bs / ms},
            {ks / mu, -(ku + ks) / mu, bs / mu, -(bs + bu) / mu}
        },
        {"x1", "x2", "x1_dot", "x2_dot"},

        // Control plant
        {
            {0},
            {0},
            {1.0 / ms},
            {-1.0 / mu}
        },
        {"F"},

        // Disturbance plant
        {
            {0, 0},
            {0, 0},
            {0, 0},
            {ku / mu, bu / mu}
        },
        // Reference change
        {
            {-1.0, 0.0},
            {-1.0, 0.0},
            {0.0, -1.0},
            {0.0, -1.0}
        },
        {"xg", "xg_dot"}
    };

    // Define initial conditions
    float state[NUM_STATES] = {0.0, 0.0, 0.0, 0.0};

    // Define simulation times (seconds)
    float dt = 0.0001;
    float t0 = 0;
    float tf = 40;

    // Define state names
    const char* stateNames[NUM_STATES] = {"x1", "x2", "x1_dot", "x2_dot"};

    // Solvers
    Solver solver;

    // Select controller
    Controller* controller = create_pid_controller(-10000.0, 0.0, -1000.0, dt);
    // Controller* controller = create_pid_controller(0.0, 0, 0, dt);

    // Select disturbance and solver type
    // init_solver(&solver, &model, state, t0, tf, dt, no_disturbance, forward_integration, controller);
    // init_solver(&solver, &model, state, t0, tf, dt, sinusoidal_disturbance, forward_integration, controller);
    init_solver(&solver, &model, state, t0, tf, dt, trapezoidal_wave_disturbance, forward_integration, controller);

    // Open file for logging
    FILE* file = fopen(output_file, "w");
    if (!file) {
        perror("Error opening file");
        return 1;
    }

    // Run simulation
    run_forward_integration_solver(&solver, file);

    // Free controller data
    controller->free_controller_data(controller->controller_data);

    fclose(file);
    return 0;
}