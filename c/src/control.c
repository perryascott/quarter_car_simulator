#include "control.h"
#include "model.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// PID Control Law Function
float* pid_control_law(Model* model, float* state, void* data) {
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
    float sprung_mass_accel = 0.0f;
    if (pid_data->prev_state != NULL) { // Only calculate if prev_state is initialized
        sprung_mass_accel = (state[2] - pid_data->prev_state[2]) / pid_data->dt; // Corrected: use velocity difference for acceleration
    } else {
        pid_data->prev_state = (float*)malloc(sizeof(float) * NUM_STATES);
        if (pid_data->prev_state == NULL) {
            perror("Memory allocation failed for prev_state");
            exit(1);
        }
    }

    // Calculate error terms
    float error = sprung_mass_accel;
    pid_data->error_integral += error * pid_data->dt;
    float error_derivative = (error - pid_data->error_prev) / pid_data->dt;
    pid_data->error_prev = error; // Update last error

    // Calculate PID
    control_input[0] = pid_data->p * error + pid_data->i * pid_data->error_integral + pid_data->d * error_derivative;

    // Set prev_state to current state
    memcpy(pid_data->prev_state, state, sizeof(float) * NUM_STATES);

    // Constrain to min and max
    // control_input[0] = fmin(fmax(-2000.0, control_input[0]),2000.0); // Clip at 1000 Newtons

    return control_input;
}

void free_pid_controller_data(void* data) {
    PIDControllerData* pid_data = (PIDControllerData*)data;
    free(pid_data->prev_state);
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

    pid_data->p = p;
    pid_data->i = i;
    pid_data->d = d;
    pid_data->dt = dt;
    pid_data->error_integral = 0.0f;
    pid_data->error_prev = 0.0f;
    pid_data->prev_state = NULL;

    controller->control_law = pid_control_law; // Assign the PID control law function
    controller->controller_data = pid_data;    // Store the PID data
    controller->free_controller_data = free_pid_controller_data;

    return controller;
}