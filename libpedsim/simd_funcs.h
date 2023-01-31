#include <emmintrin.h>
#include "ped_agent.h"

namespace Ped {
    class Simd_funcs
    {
        public:
            Simd_funcs(int n_agents);
            void update_pos(int n_agents, std::vector<Ped::Tagent*> agents);
            //void update_dest(float length, Ped::Tagent* agent);

        private:
            float *xPos;
            float *yPos;
            float *xDest;
            float *yDest; 
    };
}