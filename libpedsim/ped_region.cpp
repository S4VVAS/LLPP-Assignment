#include "ped_region.h"
#include <omp.h>
#include <iostream> //TODO: REMOVE

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
    agents = std::vector<Tagent*>(incoming.begin(), incoming.end());
    incoming = list<Tagent*>();
}

std::vector<Ped::Tagent*> Ped::region::getAgents()
{
    return agents;
}

bool Ped::region::isInRegion(int x, int y)
{
    if (x >= x1 && x < x2 &&
		y >= y1 && y < y2)
        return true;
    return false;
}

void Ped::region::splitRegion(int size, float threshold)
{
    if (hasSubRegions())
    {
        splitLeft->splitRegion(size, threshold);
        splitRight->splitRegion(size, threshold);
        return;
    }

    if ((depth >= maxDepth) || (agents.size() / (float) size) <= threshold)
        return;
    // Check which side to split in half
    std::cout << "Agents in region: " << agents.size() << ", region split in half!" << std::endl; // TODO: REMOVE

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
    // Commit incoming agents to both regions
    splitLeft->replace();
    splitRight->replace();
    // Clear incoming list of this region
    std::cout << "this size: " << incoming.size() << ", left: " << splitLeft->getAgents().size() << ", right:" << splitRight->getAgents().size() << std::endl;
    incoming = list<Tagent*>();


}

bool Ped::region::hasSubRegions()
{
    return (splitLeft != NULL);
}