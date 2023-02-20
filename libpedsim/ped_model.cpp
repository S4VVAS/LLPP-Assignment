//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include "cuda_testkernel.h"
#include <omp.h>
#include <thread>
#include <pthread.h>
#include <stdlib.h>

#define SCREEN_WIDTH 160 // How many cells wide the field is
#define SCREEN_HEIGHT 120 // How many cells high the field is

// For load-balancing
#define SPLIT_THRESHOLD 0.25 // Threshold for splitting regions (for load-balancing)
#define MAX_DEPTH 2 // Maximum depth of a region (for load-balancing)
#define N_REGIONS 4 // Number of regions, keep this in a squared number

// TODO: Move these two into the class definitions!
bool COLLISIONS = false;
bool LOADBALANCING = true;


void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario, IMPLEMENTATION implementation, bool collisions)
{
	// Convenience test: does CUDA work on this machine?
	cuda_test();

	// Set 
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());

	// Set up destinations
	destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	// Sets the chosen implemenation. Standard in the given code is SEQ
	this->implementation = implementation;
	
	// Check whether to use collisions (assignment 3)
	COLLISIONS = collisions;
	if (COLLISIONS)
		setupRegions();

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();

	SIMD = NULL;
	if (implementation == Ped::VECTOR)
		SIMD = new Simd_funcs(agents);
	
	if (implementation == Ped::CUDA)
		gpu_funcs = new Gpu_funcs(agents);
}

// Set up regions for assigment 3
void Ped::Model::setupRegions()
{
	// Create regions
	unsigned int xRegions = N_REGIONS / 2; // How many regions in x-coordinate
	unsigned int yRegions = N_REGIONS / 2; // How many regions in y-coordinate
	// Divide screen into regions in width (x) and height (y)
	for (int i = 0; i < xRegions; i++)
	{
		for (int j = 0; j < yRegions; j++)
		{
			// Get coordinates for each region
			int x1 = i*(SCREEN_WIDTH / xRegions);
			int x2 = (i+1)*(SCREEN_WIDTH / xRegions); 
			int y1 = j*(SCREEN_HEIGHT / yRegions);
			int y2 = (j+1)*(SCREEN_HEIGHT / yRegions);
			// Att region and fill it up with agents
			Ped::region *r = new Ped::region(x1,x2,y1,y2, 0, MAX_DEPTH);
			for (Ped::Tagent* agent : agents) 
			{ 
				// Add agents and assign to region
				r->add(agent);
			}
			r->replace();
			regions.push_back(r);
		}
	}
}

// Argument bundle for Pthreads
struct args 
{
    int start;
	int end;
	std::vector<Ped::Tagent*> *agents;
};

// Move agents-task for Pthread-implementation
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


void Ped::Model::handleRegionalAgents(Ped::region *r)
{
	// If there are sub-regions, run these recursivly
	if (r->hasSubRegions())
	{
		handleRegionalAgents(r->splitLeft);
		handleRegionalAgents(r->splitRight);
	}
	else
	{
		std::vector<Tagent*> localAgents = r->getAgents(); // Agents within region
		if (localAgents.size() > 0)
		#pragma omp task
		{
			// Move each agent within a region.
			// If it cannot be moved within a region, push it to the otherRegions stack
			for (Ped::Tagent* agent : localAgents)
			{ 
				agent->computeNextDesiredPosition();
				// If the agent is not in the region move directly to otherRegions-stack
				// An optimization
				if (!r->isInRegion(agent->getDesiredX(), agent->getDesiredY())) 
					r->outgoing.push(agent);
				else
				{
					// If move is successful (within region bounds) - add to the region, otherwise push to outgoing agents
					if (move(agent, true, r))
						r->add(agent);
					else
						r->outgoing.push(agent);
				}
			}			

		}
	}
}

void Ped::Model::handleOutgoingAgents(Ped::region *r)
{
	// If there are sub-regions, run these recursivly
	if (r->hasSubRegions())
	{
		handleOutgoingAgents(r->splitLeft);
		handleOutgoingAgents(r->splitRight);
	}
	else
	{
		// Pop the whole stack of all outgoing agents and place them in their regions
		while (!r->outgoing.empty())
		{
			Ped::Tagent *agent = r->outgoing.top(); 
			move(agent, false, r); // move and check collisions with all agents since we move between regions
			// Add to all regions - the "region->add"-funcion checks whether the agent is within its bounds automatically  
			for (int j = 0; j < regions.size(); j++)
			{
				regions[j]->add(agent); // sub-regions will be handled recursivly inside "add"
			}
			r->outgoing.pop();
		}
	}

}

int const tNum = 8; // Number of threads for Pthreads

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
				 if (COLLISIONS)
				 {
					 move(agent, false, regions[0]);
				 }
				 else
				 {
					 agent->setX(agent->getDesiredX());
					 agent->setY(agent->getDesiredY());
				 }


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
			if (COLLISIONS)
				 {
					#pragma omp parallel
					{
						#pragma omp single
						{ 
							// Agents that are changing region
							for (int i = 0; i < regions.size(); i++)
							{
								handleRegionalAgents(regions[i]);
							}
							#pragma omp taskwait
                            // Take care of transitioning agents
                            for (int i = 0; i < regions.size(); i++)
                            {
								handleOutgoingAgents(regions[i]);
                            }
							#pragma omp parallel for
							for (int i = 0; i < regions.size(); i++)
                            {
								if (LOADBALANCING)
								{
									regions[i]->splitRegion(agents.size(), SPLIT_THRESHOLD);
									regions[i]->replace();
								}
								else
									regions[i]->replace();
							}
						}	
					}
				 }
				 else
				 {
					#pragma omp parallel for
					for (Ped::Tagent* agent : agents)
					{
						agent->computeNextDesiredPosition();
						agent->setX(agent->getDesiredX());
						agent->setY(agent->getDesiredY());
					}		
				}
			// We argue that we don't have to point out shared or private variables in this case
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
bool Ped::Model::move(Ped::Tagent *agent, bool collisions, Ped::region *r)
{
	// Search for neighboring agents
	// If we're using collions, only check the current region
	set<const Ped::Tagent *> neighbors = (collisions) 
		? getNeighbors(agent->getX(), agent->getY(), 2, r->getAgents())
		: getNeighbors(agent->getX(), agent->getY(), 2, agents);
	 
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

	bool foundPosition = false;
	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			if (collisions)
			{
				if (!r->isInRegion((*it).first, (*it).second))
					continue;
			}

			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);
			foundPosition = true;
			break;
		}
	}

	return foundPosition;
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist, std::vector<Tagent*> region) const {

	// reference version returns list of all neighbors
	//if (!COLLISIONS)
	//	return set<const Ped::Tagent*>(agents.begin(), agents.end());
	
	// Our version checks manhattan distance
	set<const Ped::Tagent*> neighbors;
	for ( Ped::Tagent* agent : region)
	{

		if (abs(agent->getX() - x) < dist && abs(agent->getY() - y) < dist)
			neighbors.insert(agent);
	}		
	return neighbors;
}

void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });
}
