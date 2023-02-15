#include "ped_region.h"

Ped::region::region(int x1, int x2, int y1, int y2)
{
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
    incoming = new stack<Tagent*>();
    //agents   = new std::vector<Tagent*>();
}

bool Ped::region::update_position(Ped::Tagent *agent, int x, int y)
{
    return false;
}

bool Ped::region::add(Ped::Tagent *agent)
{
    //agents.push_back(agent);
    return true;
}

bool Ped::region::remove(Ped::Tagent *agent)
{
    return false;
}