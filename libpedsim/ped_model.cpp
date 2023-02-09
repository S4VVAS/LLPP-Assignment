//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include "cuda_testkernel.h"
#include <omp.h>
#include <thread>
#include <pthread.h>


#include <stdlib.h>

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario, IMPLEMENTATION implementation)
{
	// Convenience test: does CUDA work on this machine?
	cuda_test();

	// Set 
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());

	// Set up destinations
	destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	// Sets the chosen implemenation. Standard in the given code is SEQ
	this->implementation = implementation;

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();

	SIMD = NULL;
	if (implementation == Ped::VECTOR)
		SIMD = new Simd_funcs(agents);
	
	if (implementation == Ped::CUDA)
		gpu_funcs = new Gpu_funcs(agents);
}

struct args 
{
    int start;
	int end;
	std::vector<Ped::Tagent*> *agents;
};

void *moveAgent(void *input)
{
	struct args *args2 = (struct args*)input;
    for(int i = args2->start ; i < args2->end; i++)
    {
		Ped::Tagent* agent = (*args2->agents).at(i);
        agent->computeNextDesiredPosition();
		agent->setX(agent->getDesiredX());
		agent->setY(agent->getDesiredY());
	}
	delete args2;
    pthread_exit(NULL);
}


int const tNum = 10; // Number of threads

void Ped::Model::tick()
{
	switch(implementation){
		case Ped::PTHREAD : 
		{
				pthread_t threads[tNum];
				int step = (agents.size() - (agents.size() % tNum)) / tNum ;
				int curr = 0;

				for(int threadNum = 1; threadNum <= tNum; threadNum++) 
				{
					// Set up the structure to send with the task
					struct args *arguments = new args;
					arguments->start = curr;
					arguments->agents = &agents;
					
					// Divide task for threads
					if(curr + step + step >= agents.size())
						arguments->end = agents.size();
					else
						arguments->end = curr + step;

					curr = arguments->end;     

					if(pthread_create(&threads[threadNum], NULL, moveAgent, arguments)) 
					{
							printf("Error: thread was not created\n");
							exit(EXIT_FAILURE);
					}
					
				}
				// Spawn the threads
				for(int threadNum = 1; threadNum <= tNum; threadNum++) {     
					pthread_join(threads[threadNum], NULL);
				}
				
				break;
			}
			 
		case Ped::SEQ : 
		{
			 for ( Ped::Tagent* agent : agents)
			 {
    
				agent->computeNextDesiredPosition();

				agent->setX(agent->getDesiredX());
				agent->setY(agent->getDesiredY());

			}
				 
			 break;
		}
		case Ped::CUDA : {
			gpu_funcs->update_pos();
			
			break;
			}
		case Ped::VECTOR : 
		{
			SIMD->update_pos();

			break;
		}
		case Ped::OMP : 
		{
			// - Uncomment to set n of threads
			//omp_set_num_threads(8);
			#pragma omp parallel for
			// We argue that we don't have to point out shared or private variables in this case
			for ( Ped::Tagent* agent : agents)
			{
					agent->computeNextDesiredPosition();

					agent->setX(agent->getDesiredX());
					agent->setY(agent->getDesiredY());
			}		
			break;
		}
	}
	
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->getX();
	int diffY = pDesired.second - agent->getY();
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->getY());
		p2 = std::make_pair(agent->getX(), pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

			break;
		}
	}
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {

	// create the output list
	// ( It would be better to include only the agents close by, but this programmer is lazy.)	
	return set<const Ped::Tagent*>(agents.begin(), agents.end());
}

void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });
}
