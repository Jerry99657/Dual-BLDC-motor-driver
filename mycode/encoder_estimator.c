#include "encoder_estimator.h"

static void reset_velocity(EncoderEstimator *state)
{
    for (unsigned i=0; i<ENCODER_VELOCITY_WINDOW; ++i)
        state->history[i]=state->position;
    state->index=state->count=0;
    state->velocity=0.0f;
}

void EncoderEstimator_Reset(EncoderEstimator *state, float angle,
                            uint32_t samples, bool synced)
{
    *state=(EncoderEstimator){.previous=angle, .samples=samples, .synced=synced};
}

bool EncoderEstimator_Update(EncoderEstimator *state, float angle,
                             uint32_t samples, bool healthy, int direction,
                             float period)
{
    if (!healthy || period<=0.0f) {
        state->synced=false;
        reset_velocity(state);
        return false;
    }
    if (samples==state->samples) return false;
    state->samples=samples;
    if (!state->synced) {
        state->previous=angle;
        state->synced=true;
        reset_velocity(state);
        return true;
    }
    float delta=angle-state->previous;
    if (delta>3.14159265358979323846f) delta-=6.28318530717958647692f;
    else if (delta< -3.14159265358979323846f) delta+=6.28318530717958647692f;
    state->previous=angle;
    state->position+=delta*(float)direction;

    float old=state->history[state->index];
    state->history[state->index]=state->position;
    state->index=(state->index+1U)%ENCODER_VELOCITY_WINDOW;
    if (state->count<ENCODER_VELOCITY_WINDOW) ++state->count;
    state->velocity=(state->position-old)/(period*(float)state->count);
    return true;
}
