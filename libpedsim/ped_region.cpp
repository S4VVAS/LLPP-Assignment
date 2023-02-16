#include "ped_region.h"

Ped::region::region(int x1, int x2, int y1, int y2)
{
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
    incoming = stack<Tagent*>();
    agents   = std::vector<Tagent*>();
}

bool Ped::region::update_position(Ped::Tagent *agent, int x, int y)
{
    return false;
}

bool Ped::region::add(Ped::Tagent *agent)
{
    if (isInRegion(agent->getX(), agent->getY()))
        {
            agents.push_back(agent);
            return true;
        }
    return false;
}

bool Ped::region::remove(Ped::Tagent *agent)
{
    return false;
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