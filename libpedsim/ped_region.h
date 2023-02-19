#ifndef _ped_region_h_
#define _ped_region_h_

#include "ped_model.h"
#include <list>
#include <vector>
//#include <stdatomic.h>

namespace Ped
{
    class Tagent;
    
    class region
    {
        public:
            region(int x1, int x2, int y1, int y2);
            // Add an agent to the region
            bool add(Tagent *agent);
            void replace();
            // Remove an agent from the region
            //bool remove(Tagent *agent);
            // Checks whether a position is within a region
            bool isInRegion(int x, int y);
            std::vector<Tagent*> getAgents();

            // For dynamic load balancing,
            // split region into subregions
            region *splitLeft;
            region *splitRight;

        private:
            list<Tagent*> incoming; // All possible candidates of entering the region
            std::vector<Tagent*> agents; // All the agents in the region

            // The bounds of the region
            int x1;
            int x2;
            int y1;
            int y2;
    };
}

#endif
