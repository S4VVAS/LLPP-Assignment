#ifndef _ped_region_h_
#define _ped_region_h_

#include "ped_model.h"
#include <stack>
#include <vector>
#include <tuple>
#include <map>
//#include <stdatomic.h>

namespace Ped
{
    class Tagent;
    class Model;

    class coordinates{
            public:
                int x;
                int y;

                explicit coordinates(int x, int y) : x(x) , y (y) {}

                bool operator<(const coordinates& b) const{
                    if (x < b.x) return true;
                    if (x > b.x) return false;
                    if (y < b.y) return true;
                    return false;
                }
    };
    
    class region
    {
        public:
            region(int x1, int x2, int y1, int y2, region* parent, bool horizontalCut, Ped::Model* model);
            bool update_position(Tagent *agent, int x, int y);
            // Add an agent to the region
            bool add(Tagent *agent);
            // Remove an agent from the region
            bool remove(Tagent *agent);
            // Checks whether a position is within a region
            std::vector<Tagent*> getAgents();
            bool isInRegion(int x, int y);
            bool isBusy(int x, int y);

            // TODO: make private
            // The bounds of the region
            int x1;
            int x2;
            int y1;
            int y2;

            //Dynamic specific variables
            region* parent;
            region* leftChild = NULL;
            region* rightChild = NULL;
            int limit = 50;
            bool horizontalCut;
            std::map<coordinates, Ped::Tagent *> agentMap = {};

            //Dynamic specific functions
            void giveBirth();
            void killChildren();


            




        private:
            stack<Tagent*> incoming; // All possible candidates of entering the region
            std::vector<Tagent*> agents; // All the agents in the region

            Model* model;
        

    };

    

//Dynamic regions
//A region has 2 child regions, first set to null
//When threshold is reached, children are created
//In order to avoid moving the agents from the vector change it to a hashmap, 
//  or include a parallel hashmap to the vector, this would allow O(N) itterations
//  and O(1) fetches.
//  Hashmap indexed in (x,y).
//When branching O(N) transfer agents within coords to each child
//Set new coords for the children and set parent
//Make methods to branch and debranch with respective levels
//  For instance say that limit = 100
//The switch happands atomically with get and set / CAS instr (Compare And Swap). 
//When parent has children it inactivates itself and lays dormant without removing 
//  agents from its lists (for now).



}

#endif
