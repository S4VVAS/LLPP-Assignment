#ifndef _gpu_kernel_h_
#define _gpu_kernel_h_ 1

#include "ped_agent.h"
#include "ped_waypoint.h"

int cudamain(float *d_xPos, float *d_yPos, float *d_xDest, float *d_yDest, float *d_length, int size);

#endif