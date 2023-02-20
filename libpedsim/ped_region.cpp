#include "ped_region.h"

Ped::region::region(int x1, int x2, int y1, int y2, region* parent, bool horizontalCut, Ped::Model* model)
{
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
    this->parent = parent;
    incoming = stack<Tagent*>();
    agents   = std::vector<Tagent*>();
    this->horizontalCut = horizontalCut;
    this->model = model;
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
            agentMap.insert(pair<coordinates, Ped::Tagent *>(coordinates(agent->getX(),agent->getY()), agent));
            return true;
        }
    return false;
}

bool Ped::region::remove(Ped::Tagent *agent)
{
    if(!isInRegion(agent->getX(), agent->getY())){
        for(int i = 0; i < agents.size(); i++)
            if(agents.at(i) == agent){
                agents.erase(agents.begin() + i);
                break;
            }
        agentMap.erase(coordinates(agent->getX(),agent->getY()));
        return true;
    }
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

bool Ped::region::isBusy(int x, int y)
{
    return false;
    // Use hashset to check if position is busy
}

void Ped::region::giveBirth(){
    //Initialize children
    if(horizontalCut){ //Cut horizontally
        this->leftChild = new Ped::region(x1,x2,y1,y2/2-1,(this), !horizontalCut, model);
        this->rightChild = new Ped::region(x1,x2,y2/2,y2,(this), !horizontalCut, model);
    }
    else{ //Cut vertically
        this->leftChild = new Ped::region(x1,x2/2-1,y1,y2,(this), !horizontalCut, model);
        this->rightChild = new Ped::region(x2/2,x2,y1,y2,(this), !horizontalCut, model);
    }

    //Adds agents to children
    for(int i = 0; i < agents.size(); i++){
        if(leftChild->isInRegion(agents[i]->getX(),agents[i]->getY()))
            leftChild->add(agents[i]);
        else
            rightChild->add(agents[i]);
    }

    //Add child regions to global vector
    model->addRegion(leftChild);
    model->addRegion(rightChild);

    //Remove agents from this region
    agents.clear();

}

void Ped::region::killChildren(){
    
}