#include "ped_region.h"

Ped::region::region(int x1, int x2, int y1, int y2)
{
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

/*
bool Ped::region::remove(Ped::Tagent *agent)
{
    // TODO:
    // Might be slow to use remove, since we have to iterate the whole
    // vector each time, maybe it's better to create a new vector where
    // you instead add each agent you use and replace it at the end
    // of each tick

    agents.erase(std::find(agents.begin(),agents.end(),agent)-1);

    return false;
}
*/

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