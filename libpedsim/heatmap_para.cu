// Created for Low Level Parallel Programming 2017
//
// Implements the heatmap functionality. 
//
#include "ped_model.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
using namespace std;

// Memory leak check with msvc++
#include <stdlib.h>

// TODO:
// 1. Something must use shared memory
// 2. Maybe move desiredX/desiredY to memory, but would this give speedup or the contrary?

// Sets up the heatmap
void Ped::Model::setupHeatmapPara()
{
	cudaMallocManaged(&hm, SIZE*SIZE*sizeof(int));
	cudaMallocManaged(&shm, SCALED_SIZE*SCALED_SIZE*sizeof(int));
	cudaMallocManaged(&bhm, SCALED_SIZE*SCALED_SIZE*sizeof(int));

	// These are just an abstraction to those above, using pointers
	heatmap = (int**)malloc(SIZE*sizeof(int*));
	scaled_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));
	blurred_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));

	for (int i = 0; i < SIZE; i++)
	{
		heatmap[i] = hm + SIZE*i;
	}
	for (int i = 0; i < SCALED_SIZE; i++)
	{
		scaled_heatmap[i] = shm + SCALED_SIZE*i;
		blurred_heatmap[i] = bhm + SCALED_SIZE*i;
	}

	// This replaces the calloc call
	for (int i = 0; i < SIZE * SIZE; i++)
	{
		hm[i] = 0;
	}
	/*
	for (int i = 0; i < agents.size(); i++)
	{
		agents_desiredX[i] = agents[i]->getDesiredX();
		agents_desiredY[i] = agents[i]->getDesiredY();
	}
	*/

}

__global__
void fadeOutAgentsKernel(int *heatmap)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	heatmap[x+y*SIZE] = (int)round(heatmap[x+y*SIZE] * 0.80);
}

__global__
void paintHeatmap(int *heatmap, int *blurred_heatmap)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;// - blockIdx.x * 2;
	int y = blockIdx.y * blockDim.y + threadIdx.y;// - blockIdx.y * 2;
	int const blockSize = 16;
	//int tidx = (threadIdx.x - blockIdx.x * 2) % blockSize;
	//int tidy = (threadIdx.y - blockIdx.y * 2) % blockSize;
	int tidx = threadIdx.x + 2;
	int tidy = threadIdx.y + 2;

	if (x > SCALED_SIZE || y > SCALED_SIZE)
		return;


	// Get values from the heatmap
	int const cellSize = 5;
	int hmX = x / cellSize; 
	int hmY = (y / cellSize) * SIZE;
	int value = heatmap[hmX+hmY];
	
	// The scaled heatmap - using shared memory
	int const padding = 4;
	__shared__ int sh[padding + blockSize][padding + blockSize]; // we need padding for gaussian blur
	sh[tidx][tidy] = value;
	
	// TODO: We also need the values of the padding

	// Weights for blur filter
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};

#define WEIGHTSUM 273
	// Apply gaussian blurfilter	
	int i = x + 2;
	int j = y + 2;

	if (tidx < 2 || tidx >= 14 || tidy < 2 || tidy >= 14)
		return;
	if (i >= SCALED_SIZE - 2 || j >= SCALED_SIZE - 2)
		return; 

	int sum = 0;
	for (int k = -2; k < 3; k++)
	{
		for (int l = -2; l < 3; l++)
		{
			sum += w[2 + k][2 + l] * sh[tidy + k][tidx + l];
		}
	}
	value = sum / WEIGHTSUM;
	blurred_heatmap[(j * SCALED_SIZE) + i] = 0x00FF0000 | value << 24;
}


// Updates the heatmap according to the agent positions
void Ped::Model::fadeOutAgents()
{
	dim3 threadsPerBlock(16, 16);
	dim3 numBlocks(SIZE / threadsPerBlock.x, SIZE / threadsPerBlock.y);
	fadeOutAgentsKernel<<<numBlocks, threadsPerBlock>>>(hm);
}


void Ped::Model::updateHeatmapPara()
{
	cudaDeviceSynchronize();

	// Count how many agents want to go to each location
	#pragma parallel for
	for (int i = 0; i < agents.size(); i++)
	{
		Ped::Tagent* agent = agents[i];
		int x = agent->getDesiredX();
		int y = agent->getDesiredY();

		if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
		{
			continue;
		}

		// intensify heat for better color results
		int hm_value = hm[x+y*SIZE];
		hm[x+y*SIZE] = (hm_value < (255 - 40)) ? hm_value + 40 : 255;

	}
	// Paint the heatmap
	dim3 threadsPerBlock(16, 16);
	// num blocks = SCALED_SIZE / threadsPerBlock.x + SCALED_SIZE / (threadsPerBlock.x - 2) + SCALED_SIZE / threadsPerBlock.x 
	dim3 numBlocks((SCALED_SIZE) / threadsPerBlock.x, (SCALED_SIZE) / threadsPerBlock.y);
	paintHeatmap<<<numBlocks,threadsPerBlock>>>(hm, bhm);
	cudaDeviceSynchronize();
}
