#ifndef ENCODER_ESTIMATOR_H
#define ENCODER_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

/* One instance per axis. Input angles are already in the sensor driver's
 * convention; direction is the independently calibrated +1/-1 for that axis.
 * The 1 kHz owner admits fresh sample counters only. Filtering and electrical
 * alignment remain with the controller (including the M1 encoder/HSO mux).
 * Velocity is mechanical rad/s from the unchanged four-point difference
 * (4 ms once filled); this module adds no prediction, PLL or extra delay. */
enum { ENCODER_VELOCITY_WINDOW = 4 };
typedef struct {
    float previous, position, velocity, history[ENCODER_VELOCITY_WINDOW];
    uint32_t samples;
    uint8_t index, count;
    bool synced;
} EncoderEstimator;

/* Alignment/profile changes establish mechanical zero. A cold or unhealthy
 * instance instead rebases on its first fresh healthy frame, retaining position
 * across an outage and excluding the unknown displacement from velocity. */
void EncoderEstimator_Reset(EncoderEstimator *state, float angle,
                            uint32_t samples, bool synced);
/* True means a fresh healthy frame was accepted (count==0 on recovery).
 * Duplicate frames leave the estimate unchanged; unhealthy input immediately
 * clears speed and window readiness. period is the existing outer-loop step,
 * not the 10 kHz SPI/DMA period. Getters must never advance this state. */
bool EncoderEstimator_Update(EncoderEstimator *state, float angle,
                             uint32_t samples, bool healthy, int direction,
                             float period);
#endif
