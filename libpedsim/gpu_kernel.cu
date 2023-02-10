#include "gpu_kernel.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#define BLOCK_WIDTH 128
#define N_BLOCKS 4

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
        int x = d_xPos[core];
        int y = d_yPos[core];
        float diffX = d_xDest[core] - x;
        float diffY = d_yDest[core] - y;
        float len = sqrt(diffX * diffX + diffY * diffY);
        d_xPos[core] = round(x + diffX / len);
        d_yPos[core] = round(y + diffY / len);
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