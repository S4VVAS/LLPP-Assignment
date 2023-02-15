#ifndef _ped_region_h_
#define _ped_region_h_

#include "ped_model.h"
#include <stack>
//#include <stdatomic.h>

namespace Ped
{
    class region
    {
        public:
            region();
            bool update_position(Tagent agent, int x, int y);
            bool add(Tagent agent, int x, int y);
            bool remove(Tagent agent);

        private:
            //std::_Atomic int counter;
            stack<Tagent*> *incoming;
            //std::vector<Tagent*> agents;
            //std::hash
            int x1;
            int x2;
            int y1;
            int y2;
    };
}

#endif
