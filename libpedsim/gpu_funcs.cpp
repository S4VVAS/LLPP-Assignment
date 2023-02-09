#include "gpu_funcs.h"
#include "gpu_kernel.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <cstdlib>
cudaStream_t stream1;
cudaStream_t stream2;

namespace Ped
{
    Gpu_funcs::Gpu_funcs(std::vector<Ped::Tagent*> startAgents)
    {
        agents = startAgents;
        bytes = sizeof(float) * agents.size();
        
        // Init host data
        cudaMallocHost((void**)&xDest, bytes);
        cudaMallocHost((void**)&yDest, bytes);
        cudaMallocHost((void**)&xPos, bytes);
        cudaMallocHost((void**)&yPos, bytes);
        cudaMallocHost((void**)&length, bytes);

        // Set up the start-positions and start destinations
        for (int i = 0; i <  agents.size(); i++)
        {
            xPos[i]  = (float) agents.at(i)->getX();
            yPos[i]  = (float) agents.at(i)->getY();
            agents.at(i)->computeNextDesiredPosition();
            if (agents[i]->getDestination() == NULL)
                continue;
            xDest[i] = (float) agents.at(i)->getDestination()->getx();
            yDest[i] = (float) agents.at(i)->getDestination()->gety();
        }

        // Init arrays of graphics memory
        cudaMalloc(&d_xPos,bytes);
        cudaMalloc(&d_yPos,bytes);
        cudaMalloc(&d_xDest,bytes);
        cudaMalloc(&d_yDest,bytes);
        cudaMalloc(&d_length,bytes);

        // Move data to graphics memory
        cudaMemcpy(d_xPos,  xPos,  bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(d_yPos,  yPos,  bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(d_xDest, xDest, bytes, cudaMemcpyHostToDevice);
        cudaMemcpy(d_yDest, yDest, bytes, cudaMemcpyHostToDevice);

        // Setting up streams
        cudaStreamCreate(&stream1);
        cudaStreamCreate(&stream2);
    }

    void Gpu_funcs::update_pos()
    {
        // Do calculations in graphic units
        cudamain(d_xPos, d_yPos, d_xDest, d_yDest, d_length, agents.size());

        // Move data back to main memory
        cudaMemcpyAsync(length, d_length,  bytes, cudaMemcpyDeviceToHost, stream1);
        cudaMemcpyAsync(xPos,   d_xPos,    bytes, cudaMemcpyDeviceToHost, stream2);
        cudaMemcpyAsync(yPos,   d_yPos,    bytes, cudaMemcpyDeviceToHost, stream2);
        
        cudaStreamSynchronize(stream1);

        #pragma omp parallel for
        for (int i = 0; i < agents.size(); i++)
        {
            // Do branching and check for new destinations
            Ped::Twaypoint *waypoint = agents[i]->getNextDestinationSIMD(length[i]); 
            if (waypoint != NULL)
            {
                xDest[i] = (float) waypoint->getx();
                yDest[i] = (float) waypoint->gety();
            }
        }
       
        // Move new destination information to graphics memory
        cudaMemcpyAsync(d_xDest, xDest, bytes, cudaMemcpyHostToDevice, stream1);
        cudaMemcpyAsync(d_yDest, yDest, bytes, cudaMemcpyHostToDevice, stream1);
        
        cudaStreamSynchronize(stream2);

        #pragma omp parallel for
        for (int i = 0; i < agents.size(); i++)
        {
            agents[i]->setX((int) xPos[i]);
            agents[i]->setY((int) yPos[i]);
        }

        cudaStreamSynchronize(stream1);
    
    }
}
