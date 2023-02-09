#ifndef _gpu_funcs_h_
#define _gpu_funcs_h_ 1

#include "ped_agent.h"
#include "ped_waypoint.h"

namespace Ped {
    class Gpu_funcs {
        public:
            Gpu_funcs(std::vector<Ped::Tagent*> startAgents);
            void update_pos();

        private:
            float *xPos;
            float *yPos;
            float *xDest;
            float *yDest; 
            float *length; 
            float *d_xPos;
            float *d_yPos;
            float *d_xDest;
            float *d_yDest; 
            float *d_length; 
            int bytes;
            std::vector<Ped::Tagent*> agents;

    };
}

#endif