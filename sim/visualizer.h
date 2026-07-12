#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "plant.h" // State struct ke liye

void visualizer_init(void);
void visualizer_render(const quad_state_t *state);
void visualizer_close(void);

#endif