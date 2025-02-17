#include "control.h"
#include "model.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ERROR_LOW_PASS_ALPHA 0.01

// PID Control Law Function
float* pid_jerk_minimization(Model* model, float* state, float* disturbance, void* data) {
    PIDControllerData* pid_data = (PIDControllerData*)data; // Cast to PID data

    float* control_input = (float*)malloc(sizeof(float) * NUM_CONTROL_INPUTS);
    if (control_input == NULL) { // Check malloc success
        perror("Memory allocation failed in pid_control_law");
        exit(1);
    }

    // Initialize control input to zero
    for (int i = 0; i < NUM_CONTROL_INPUTS; i++) {
        control_input[i] = 0.0f;
    }

    // Calculate vertical acceleration on sprung mass from state and prev_state
    float sprung_mass_jerk = 0.0f; // derivative of acceleration
    if (pid_data->states_saved >= 2) {
        // const float v1_dot = (pid_data->prev_state[2] - pid_data->prev_prev_state[2]) / pid_data->dt;
        // const float v2_dot = (state[2] - pid_data->prev_state[2]) / pid_data->dt;
        // sprung_mass_jerk = (v2_dot - v1_dot) / pid_data->dt;
        sprung_mass_jerk = (state[2] - 2*pid_data->prev_state[2] + pid_data->prev_prev_state[2]) / (pid_data->dt * pid_data->dt); 
    } else {
        pid_data->states_saved++;
    }

    // Calculate error terms
    float error = (sprung_mass_jerk * ERROR_LOW_PASS_ALPHA) + (pid_data->error_prev * (1 - ERROR_LOW_PASS_ALPHA)) ;
    pid_data->error_integral += error * pid_data->dt;
    float error_derivative = (error - pid_data->error_prev) / pid_data->dt;
    pid_data->error_prev = error; // Update last error

    // Calculate PID
    control_input[0] = pid_data->p * error + pid_data->i * pid_data->error_integral + pid_data->d * error_derivative;
    control_input[0] -= 200.0 * state[2];

    // Set previous states
    memcpy(pid_data->prev_prev_state, pid_data->prev_state, sizeof(float) * NUM_STATES);
    memcpy(pid_data->prev_state, state, sizeof(float) * NUM_STATES);

    // Constrain to min and max
    control_input[0] = fmin(fmax(-1000.0, control_input[0]),1000.0); // Clip at 1000 Newtons

    return control_input;
}

// PID Control Law Function
float* pid_zeroth_order(Model* model, float* state, float* disturbance, void* data) {
    PIDControllerData* pid_data = (PIDControllerData*)data; // Cast to PID data

    float* control_input = (float*)malloc(sizeof(float) * NUM_CONTROL_INPUTS);
    float* rel_state = (float*)malloc(sizeof(float) * NUM_CONTROL_INPUTS);
    if (control_input == NULL || rel_state == NULL) { // Check malloc success
        perror("Memory allocation failed in pid_control_law");
        exit(1);
    }

    // Populate let state by subtracting disturbance
    for (size_t i = 0; i < NUM_STATES; i++) {
        rel_state[i] = state[i];
        for (size_t j = 0; j < NUM_DISTURBANCE_STATES; j++) {
            rel_state[i] += model->disturbance_reference_change[i][j] * disturbance[j];
        }
    }

    // Initialize control input to zero
    for (int i = 0; i < NUM_CONTROL_INPUTS; i++) {
        control_input[i] = 0.0f;
    }

    // Calculate error terms
    float error = rel_state[0];
    pid_data->error_integral += error * pid_data->dt;
    float error_derivative = (error - pid_data->error_prev) / pid_data->dt;
    pid_data->error_prev = error; // Update last error

    // Calculate PID
    control_input[0] = pid_data->p * error + pid_data->i * pid_data->error_integral + pid_data->d * error_derivative;

    // Constrain to min and max
    control_input[0] = fmin(fmax(-1000.0, control_input[0]),1000.0); // Clip at 1000 Newtons

    return control_input;
}

void free_pid_controller_data(void* data) {
    PIDControllerData* pid_data = (PIDControllerData*)data;
    free(pid_data->prev_state);
    free(pid_data->prev_prev_state);
    free(data);
}

// Function to create a PID controller
Controller* create_pid_controller(float p, float i, float d, float dt) {

    Controller* controller = (Controller*)malloc(sizeof(Controller));
    if (controller == NULL) {
        perror("Controller memory allocation failed");
        exit(1);
    }

    PIDControllerData* pid_data = (PIDControllerData*)malloc(sizeof(PIDControllerData));
    if (pid_data == NULL) {
        perror("PIDController memory allocation failed");
        free(controller); 
        exit(1);
    }

    // Gains
    pid_data->p = p;
    pid_data->i = i;
    pid_data->d = d;

    // DT
    pid_data->dt = dt;

    // Error tracking
    pid_data->error_integral = 0.0f;
    pid_data->error_prev = 0.0f;

    // Previous state tracking
    pid_data->prev_state = (float*)malloc(sizeof(float) * NUM_STATES);
    pid_data->prev_prev_state = (float*)malloc(sizeof(float) * NUM_STATES);
    pid_data->states_saved = 0;

    // Base class
    controller->control_law = pid_zeroth_order; // Assign the PID control law function
    controller->controller_data = pid_data;    // Store the PID data
    controller->free_controller_data = free_pid_controller_data;

    return controller;
}