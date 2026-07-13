#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "plant.h" // for State struct 

void visualizer_set_title(const char* title) ;
void visualizer_init(void);
void visualizer_render(const quad_state_t *state);
void visualizer_close(void);
#endif