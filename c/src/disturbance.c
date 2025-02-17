#include "model.h"
#include "disturbance.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

float* sinusoidal_disturbance(float t) {

    // Return size check
    if (NUM_DISTURBANCE_STATES > 2) {
        fprintf(stderr, "Error: sinusoidal_disturbance called with inappropriate disturbance size (%zu > 2)\n", NUM_DISTURBANCE_STATES); // Use stderr for errors
        exit(1);
    }

    float* dist = (float*)malloc(sizeof(float) * 2);
    if (dist == NULL) {  // Check malloc success
        perror("Error: Memory allocation failed in sinusoidal_disturbance");
        exit(1);
    }

    // Define parameters for the sine wave (you can adjust these)
    float amplitude = 0.05f;  // Amplitude of the sine wave
    float frequency = 5.0f;  // Frequency of the sine wave (Hz)
    float phase_offset = 0.0f; // Phase offset (radians)

    // Calculate 0th order disturbance (e.g., position/displacement)
    dist[0] = amplitude * sinf(2.0f * M_PI * frequency * t + phase_offset);

    // Calculate 1st order disturbance (e.g., velocity) - derivative of position
    dist[1] = amplitude * 2.0f * M_PI * frequency * cosf(2.0f * M_PI * frequency * t + phase_offset);

    return dist;
}

float* trapezoidal_wave_disturbance(float t) {
    if (NUM_DISTURBANCE_STATES > 2) {
        fprintf(stderr, "Error: trapezoidal_wave_disturbance called with inappropriate disturbance size (%zu > 2)\n", NUM_DISTURBANCE_STATES);
        exit(1);
    }

    float* dist = (float*)malloc(sizeof(float) * 2);
    if (dist == NULL) {
        perror("Error: Memory allocation failed in trapezoidal_wave_disturbance");
        exit(1);
    }

    // Parameters (adjust these)
    float amplitude = 0.2f;      // Amplitude of the wave
    float frequency = 0.05f;      // Frequency (Hz)
    float rise_time = 0.5f;     // Time to reach full rise
    float plateau_time = 6.0f;   // Time at max/min amplitude (seconds)

    // Calculate the period of the wave
    float period = 1.0f / frequency;

    // Calculate the time within the current period (0 to period)
    float time_in_period = fmodf(t, period); // fmodf is for floats

    // Calculate the 0th order disturbance (position)
    float position = 0.0f;
    if (time_in_period < rise_time) {
        position = amplitude * time_in_period / rise_time; // Rising slope
    } else if (time_in_period < rise_time + plateau_time) {
        position = amplitude; // Plateau
    } else if (time_in_period < 2 * rise_time + plateau_time) {
        position = amplitude * (1.0f - (time_in_period - rise_time - plateau_time) / rise_time); // Falling slope
    } else {
        position = 0.0f; // Zero
    }

    dist[0] = position;

    // Calculate the 1st order disturbance (velocity) - derivative of position
    float velocity = 0.0f;
    if (time_in_period < rise_time) {
        velocity = amplitude / rise_time; // Positive slope
    } else if (time_in_period < rise_time + plateau_time) {
        velocity = 0.0f; // Zero slope on plateau
    } else if (time_in_period < 2 * rise_time + plateau_time) {
        velocity = -amplitude / rise_time; // Negative slope
    } else {
        velocity = 0.0f; // Zero
    }

    dist[1] = velocity;

    return dist;
}


float* no_disturbance(float t) {

    // Return size check
    if (NUM_DISTURBANCE_STATES > 2) {
        fprintf(stderr, "Error: sinusoidal_disturbance called with inappropriate disturbance size (%zu > 2)\n", NUM_DISTURBANCE_STATES); // Use stderr for errors
        exit(1);
    }

    float* dist = (float*)malloc(sizeof(float) * 2);
    if (dist == NULL) {  // Check malloc success
        perror("Error: Memory allocation failed in sinusoidal_disturbance");
        exit(1);
    }

    dist[0] = 0.0;
    dist[1] = 0.0;
    return dist;
}