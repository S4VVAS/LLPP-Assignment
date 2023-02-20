#include "ped_region.h"
#include <omp.h>

#define MERGE_MOD 1.20 // modifier to threshold when merging two sub-regions

Ped::region::region(int x1, int x2, int y1, int y2, short depth, short maxDepth)
{
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
    this->depth = depth;
    this->maxDepth = maxDepth;
    incoming = list<Tagent*>();
    agents   = std::vector<Tagent*>();
    outgoing = std::stack<Tagent*>();

    splitLeft = NULL;
    splitRight = NULL;
}

bool Ped::region::add(Ped::Tagent *agent)
{
    if (hasSubRegions())
    {
        splitLeft->add(agent);
        splitRight->add(agent);
        return true;
    }
    if (isInRegion(agent->getX(), agent->getY()))
        {
            incoming.push_front(agent);
            return true;
        }
    return false;
}

void Ped::region::replace()
{
    if (hasSubRegions())
    {
        splitLeft->replace();
        splitRight->replace();
        return;
    }
    // Create subtasks if we're below the first level of depth
    if (depth > 0)
    {
        #pragma omp task
        {
            agents = std::vector<Tagent*>(incoming.begin(), incoming.end());
            incoming = list<Tagent*>();
        }

    }
    else
    {
        agents = std::vector<Tagent*>(incoming.begin(), incoming.end());
        incoming = list<Tagent*>();
    }
}

std::vector<Ped::Tagent*> Ped::region::getAgents()
{
    return agents;
}

list<Ped::Tagent*> Ped::region::getIncoming()
{
    return incoming;
}

bool Ped::region::isInRegion(int x, int y)
{
    if (x >= x1 && x < x2 &&
		y >= y1 && y < y2)
        return true;
    return false;
}

int Ped::region::splitRegion(int size, float threshold)
{
    int subAgents = 0;
    if (hasSubRegions())
    {
        subAgents += splitLeft->splitRegion(size, threshold);
        subAgents += splitRight->splitRegion(size, threshold);

        // If both subregions are smaller than the threshhold, merge them again
        int mergeThreshold = threshold * MERGE_MOD; // Make merge theshold slightly larger to avoid constant merging/splitting
        if ((subAgents / (float) size) <= mergeThreshold)
            merge();

        return subAgents;
    }

    if ((depth >= maxDepth) || (incoming.size() / (float) size) <= threshold)
        return incoming.size();

    // Check which side to split in half
    if (abs(y2-y1) < abs(x2-x1))
    {
        int width = x2 - x1;
        splitLeft  = new region(x1, x1+width/2, y1, y2, depth+1, maxDepth);
        splitRight = new region(x1+width/2, x2, y1, y2, depth+1, maxDepth);
    }
    else
    {
        int height = y2 - y1;
        splitLeft  = new region(x1, x2, y1, y1+height/2, depth+1, maxDepth);
        splitRight = new region(x1, x2, y1+height/2, y2, depth+1, maxDepth);
    }
    // Add agents to subregions
    for (Tagent* agent : incoming)
    {
        splitLeft->add(agent);
        splitRight->add(agent);
    }
    // Clear agents of this region
    subAgents = incoming.size();
    incoming = list<Tagent*>();
    agents   = std::vector<Tagent*>();

    return subAgents;

}

void Ped::region::merge()
{
    // Get all agents from sub-regions
    for (Tagent *agent : splitLeft->getIncoming())
    {
        incoming.push_front(agent);
    }
    for (Tagent *agent : splitRight->getIncoming())
    {
        incoming.push_front(agent);
    }
    delete splitLeft;
    delete splitRight;
    splitLeft = NULL;
    splitRight = NULL;
}

bool Ped::region::hasSubRegions()
{
    return (splitLeft != NULL);
}