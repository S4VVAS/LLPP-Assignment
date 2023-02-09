#include "gpu_kernel.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#define BLOCK_WIDTH 256
#define N_BLOCKS 2

__global__ void cuda_update(
    float *d_xPos, 
    float *d_yPos, 
    float *d_xDest, 
    float *d_yDest,
    float *d_length, 
    int size, // Number of agents
    int extra) // how many extra agents for each cpu-core
{
    // How many extra data to process, can't expect number of agent to be binary
    

    for (int i = 1; i < extra; i++)
    {
        int core = (i * (threadIdx.x + BLOCK_WIDTH * blockIdx.x));
        if (core >= size)
            return; 
        float diffX = d_xDest[core] - d_xPos[core];
        float diffY = d_yDest[core] - d_yPos[core];
        float len = sqrt(diffX * diffX + diffY * diffY);
        d_xPos[core] = round(d_xPos[core] + diffX / len);
        d_yPos[core] = round(d_yPos[core] + diffY / len);
        d_length[core] = len;
    }
}

int cudamain(
    float *d_xPos, 
    float *d_yPos, 
    float *d_xDest, 
    float *d_yDest, 
    float *d_length, 
    int size)
{
    int extra = size / (BLOCK_WIDTH * N_BLOCKS) + 2;
    cuda_update<<<N_BLOCKS, BLOCK_WIDTH>>>(d_xPos, d_yPos, d_xDest, d_yDest, d_length, size, extra);

    return 0;
}