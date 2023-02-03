#include <emmintrin.h>
#include "ped_agent.h"
#include "ped_waypoint.h"

namespace Ped {
    class Simd_funcs
    {
        public:
            Simd_funcs(std::vector<Ped::Tagent*> agents);
            void update_pos(std::vector<Ped::Tagent*> agents);
            //void update_dest(float length, Ped::Tagent* agent);

        private:
            float *xPos;
            float *yPos;
            float *xDest;
            float *yDest; 
            std::vector<Ped::Tagent*> agents;
    };
}
