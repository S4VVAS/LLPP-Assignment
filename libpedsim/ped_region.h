#ifndef _ped_region_h_
#define _ped_region_h_

#include "ped_model.h"
#include <list>
#include <stack>
#include <vector>
//#include <stdatomic.h>

namespace Ped
{
    class Tagent;
    
    class region
    {
        public:
            region(int x1, int x2, int y1, int y2, short depth, short maxDepth);
            // Add an agent as incoming to the region if the agent
            // is within the region and return true, otherwise false.
            bool add(Tagent *agent);
            // Replace the "agents" vektor with all agents in "incoming"
            void replace();
            // Checks whether a position is within a region
            bool isInRegion(int x, int y);
            // Split or merge regions if load-balancing is used, returns how many agents are in the current subregion
            int splitRegion(int size, float threshold);

            bool hasSubRegions();
            // Get the vetor with all agents
            std::vector<Tagent*> getAgents();
            // Get the list of all incoming agents
            list<Tagent*> getIncoming();
            
            // TODO: Move to private and use getters and setters
            // For dynamic load balancing,
            // split region into subregions
            region *splitLeft;
            region *splitRight;
            std::stack<Tagent*> outgoing; // All agents possibly moving out of the region

        private:
            list<Tagent*> incoming; // All possible candidates of entering the region
            std::vector<Tagent*> agents; // All the agents in the region

            // The bounds of the region
            int x1;
            int x2;
            int y1;
            int y2;
            short depth;
            short maxDepth;

            void merge();
    };
}

#endif
