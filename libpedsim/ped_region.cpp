#include "ped_region.h"

Ped::region::region(int x1, int x2, int y1, int y2)
{
    nAgents = 0;
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
    incoming = list<Tagent*>();
    agents   = std::vector<Tagent*>();
}

bool Ped::region::add(Ped::Tagent *agent)
{
    if (isInRegion(agent->getX(), agent->getY()))
        {
            incoming.push_front(agent);
            return true;
        }
    return false;
}

void Ped::region::replace()
{
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