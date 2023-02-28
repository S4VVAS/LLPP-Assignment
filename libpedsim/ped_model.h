//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// Model coordinates a time step in a scenario: for each
// time step all agents need to be moved by one position if
// possible.
//
#ifndef _ped_model_h_
#define _ped_model_h_

#include <vector>
#include <map>
#include <set>

#include "ped_agent.h"
#include "ped_region.h"
#include "simd_funcs.h"
#include "gpu_funcs.h"

namespace Ped{
	class Tagent;
	class region;

	// The implementation modes for Assignment 1 + 2:
	// chooses which implementation to use for tick()
	enum IMPLEMENTATION { CUDA, VECTOR, OMP, PTHREAD, SEQ };

	class Model
	{
	public:

		// Sets everything up
		void setup(std::vector<Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario,IMPLEMENTATION implementation, bool collisions);

		// Coordinates a time step in the scenario: move all agents by one step (if applicable).
		void tick();

		Simd_funcs *SIMD;

		// Returns the agents of this scenario
		const std::vector<Tagent*> getAgents() const { return agents; };

		// Adds an agent to the tree structure
		void placeAgent(const Ped::Tagent *a);

		// Cleans up the tree and restructures it. Worth calling every now and then.
		void cleanup();
		~Model();

		// Returns the heatmap visualizing the density of agents
		int const * const * getHeatmap() const { return blurred_heatmap; };
		int getHeatmapSize() const;

	private:

		// Denotes which implementation (sequential, parallel implementations..)
		// should be used for calculating the desired positions of
		// agents (Assignment 1)
		IMPLEMENTATION implementation;

		// The agents in this scenario
		std::vector<Tagent*> agents;

		// The waypoints in this scenario
		std::vector<Twaypoint*> destinations;

		// Regions to divide view 
		std::vector<region*> regions; 

		// Moves an agent towards its next position
		bool move(Ped::Tagent *agent, bool collisions, region *r);

		Gpu_funcs *gpu_funcs;

		////////////
		/// Everything below here won't be relevant until Assignment 3
		///////////////////////////////////////////////

		// Returns the set of neighboring agents for the specified position
		set<const Ped::Tagent*> getNeighbors(int x, int y, int dist, std::vector<Tagent*> region) const;
		
		// Handle agents within a region
		void handleRegionalAgents(Ped::region *r);
		// Handle agents that are leaving a region
		void handleOutgoingAgents(Ped::region *r);

		void setupRegions();

		////////////
		/// Everything below here won't be relevant until Assignment 4
		///////////////////////////////////////////////

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

		// The heatmap representing the density of agents
		int ** heatmap;

		// The scaled heatmap that fits to the view
		int ** scaled_heatmap;

		// The final heatmap: blurred and scaled to fit the view
		int ** blurred_heatmap;


		void setupHeatmapSeq();
		void updateHeatmapSeq();
		
		// Parallel heatmap processing
		void setupHeatmapPara();
		void updateHeatmapPara();
		void fadeOutAgents();
		int *hm;
		int *shm;
		int *bhm;
		int *agents_desiredX;
		int *agents_desiredY;

		//void moveAgent(int start, int end);
	};
}
#endif
