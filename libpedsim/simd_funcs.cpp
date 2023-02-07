#include "simd_funcs.h"
#include <iostream>
#include <cmath>

namespace Ped
{
    Simd_funcs::Simd_funcs(std::vector<Ped::Tagent*> startAgents)
    {
        agents = startAgents;
        std::cout << "Creating\n";

        // This results in a aligned size that is alwayds dividable with 4.
        // If agents.size() = 21 we will have 3 extra cells of padding resulting in 24.
        int alignedSize = agents.size() - (agents.size() % 4) + 4;
        // Allocate floats with SSE-compatible alignment
        xPos  = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        yPos  = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        xDest = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        yDest = (float*) _mm_malloc(alignedSize * sizeof(float), 16);

	   // Set up the start-positions and start destinations

       // TODO: Dubbelkollat och de startar rätt
        for (int i = 0; i < agents.size(); i++)
        {
            xPos[i]  = (float) agents[i]->getX();
            yPos[i]  = (float) agents[i]->getY();
        }
	std::cout << "Created\n";
    }

    std::pair<int, int> Simd_funcs::getPosition(int agentN)
    {
        int x = (int) round(xPos[agentN]);
        int y = (int) round(yPos[agentN]);
        return std::make_pair(x, y);
    }

    void Simd_funcs::update_pos()
    {

        /*
        HOW IT'S DONE IN AGENT FUNCTION:
        1. Check next destination
        2. Calculate next position to move agains:

            double diffX = destination->getx() - x;
            double diffY = destination->gety() - y;
            double len = sqrt(diffX * diffX + diffY * diffY);
            desiredPositionX = (int)round(x + diffX / len);
            desiredPositionY = (int)round(y + diffY / len);
        
        */
        for (int i = 0; i < agents.size(); i += 4)
        {
            // CHEAT: TEMPORAL, REMOVE!
            // Here I use the getNextDestination from the actual agent function,
            // we should get the next position, but somehow they still move weird
            for (int j = 0; j < 4; j++)
            {
                if ((i+j) >= agents.size())
                    break;
                // Gets a waypoint
                //Twaypoint *wp = agents[i+j]->getNextDestination();
                //if (wp == NULL)
                //    return;
                // If a waypoint exist, get the x position of the waypoint
                //xDest[i+j] = (float) wp->getx();
                //yDest[i+j] = (float) wp->gety();
                agents[i+j]->computeNextDesiredPosition();
                // When there is no new position this may be null and cause a segfault
                if (agents[i+j]->getDestination() == NULL)
                    continue;
                xDest[i+j] = (float) agents[i+j]->getDestination()->getx();
                yDest[i+j] = (float) agents[i+j]->getDestination()->gety();
            }

            // Set-up SSE-variables
            __m128 XPOS;
            __m128 YPOS;
            __m128 diffX;
            __m128 diffY;
            __m128 len; 
            __m128 t0;
            __m128 t1;

            // Load values into SSE
            XPOS = _mm_load_ps(&xPos[i]);
            YPOS = _mm_load_ps(&yPos[i]);
            t0 = _mm_load_ps(&xDest[i]);
            t1 = _mm_load_ps(&yDest[i]);

            // double diffX = destination->getx() - x;
            diffX = _mm_sub_ps(t0, XPOS);
            // double diffY = destination->gety() - y;
            diffY = _mm_sub_ps(t1, YPOS);
            // sqrt(diffX * diffX + diffY * diffY)
            t0 = _mm_mul_ps(diffX, diffX);
            t1 = _mm_mul_ps(diffY, diffY);
            len = _mm_add_ps(t0, t1);
            len = _mm_sqrt_ps(len);
    
            //desiredPositionX = (int)round(x + diffX / len);
            diffX = _mm_div_ps(diffX, len);
            XPOS  = _mm_add_ps(XPOS, diffX);
            //desiredPositionY = (int)round(y + diffY / len);
            diffY  = _mm_div_ps(diffY, len);
            YPOS   = _mm_add_ps(YPOS, diffY);

            // Store the values back
            _mm_store_ps(&xPos[i], XPOS);
            _mm_store_ps(&yPos[i], YPOS);

            // Store x and y in agents. TODO: however this is bad and slow!
            // The proper way to do this is the paint() function inside the mainWindow
            for (int j = 0; j < 4; j++) 
            {
                if ((i+j) < agents.size() )
                {
                    agents[i + j]->setX((int) round(xPos[i + j]));
                    agents[i + j]->setY((int) round(yPos[i + j]));

                }
            }
        }
    }
    
    void Simd_funcs::update_dest(float length, Ped::Tagent* agent, int n)
    {
        bool agentReachedDestination = length < agent->getDestination()->getr();
        if (agentReachedDestination || agent->getDestination() == NULL) 
        {
            Ped::Twaypoint *waypoint = agent->changeDestination();
            if (waypoint != NULL)
            {
                xDest[n] = (float) waypoint->getx();
                yDest[n] = (float) waypoint->gety();
            }
        }
    }
} 
