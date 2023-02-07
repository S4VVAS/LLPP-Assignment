#include <emmintrin.h>
#include "ped_agent.h"
#include "ped_waypoint.h"

#ifndef _simd_funcs_h_
#define _simd_funcs_h_ 1

namespace Ped {
    class Simd_funcs
    {
        public:
            Simd_funcs(std::vector<Ped::Tagent*> startAgents);
            void update_pos();
            std::pair<int, int> getPosition(int agentN);

        private:
            float *xPos;
            float *yPos;
            float *xDest;
            float *yDest; 
            std::vector<Ped::Tagent*> agents;
            void update_dest(float length, int n);
    };
}

#endif