#ifndef _ped_region_h_
#define _ped_region_h_

#include "ped_model.h"
#include <stack>
#include <vector>
//#include <stdatomic.h>

namespace Ped
{
    class Tagent;
    
    class region
    {
        public:
            region(int x1, int x2, int y1, int y2);
            bool update_position(Tagent *agent, int x, int y);
            // Add an agent to the region
            bool add(Tagent *agent);
            // Remove an agent from the region
            bool remove(Tagent *agent);
            // Checks whether a position is within a region
            bool isInRegion(int x, int y);
            std::vector<Tagent*> getAgents();

        private:
            stack<Tagent*> incoming; // All possible candidates of entering the region
            std::vector<Tagent*> agents; // All the agents in the region

            // The bounds of the region
            int x1;
            int x2;
            int y1;
            int y2;
    };
}

#endif
