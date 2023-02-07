#include "simd_funcs.h"
#include <iostream>
#include <cmath>

#define SIMD_SIZE 4

namespace Ped
{
    Simd_funcs::Simd_funcs(std::vector<Ped::Tagent*> startAgents)
    {
        agents = startAgents;
        std::cout << "Creating\n";

        // This results in a aligned size that is alwayds dividable with 4.
        // If agents.size() = 21 we will have 3 extra cells of padding resulting in 24.
        int alignedSize = agents.size() - (agents.size() % SIMD_SIZE) + SIMD_SIZE;
        // Allocate floats with SSE-compatible alignment
        xPos  = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        yPos  = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        xDest = (float*) _mm_malloc(alignedSize * sizeof(float), 16);
        yDest = (float*) _mm_malloc(alignedSize * sizeof(float), 16);

	   // Set up the start-positions and start destinations
        for (int i = 0; i < agents.size(); i++)
        {
            xPos[i]  = (float) agents[i]->getX();
            yPos[i]  = (float) agents[i]->getY();
            agents[i]->computeNextDesiredPosition();
            if (agents[i]->getDestination() == NULL)
                continue;
            xDest[i] = (float) agents[i]->getDestination()->getx();
            yDest[i] = (float) agents[i]->getDestination()->gety();
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
        for (int i = 0; i < agents.size(); i += SIMD_SIZE)
        {
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


            // Check for each agent if it has reached its destination and change waypoint.
            // We need the length from current position to destination for this.
            float storedLength[SIMD_SIZE];
            _mm_store_ps(storedLength, len);
            for (int j = 0; j < SIMD_SIZE; j++) 
            {
                if ((i+j) < agents.size() )
                {
                    // TODO: Remove this somehow! Right now we're setting position inside agent!
                    // Should be set when draws, probably in paint() in mainWindow.cpp
                    agents[i + j]->setX((int) round(xPos[i + j]));
                    agents[i + j]->setY((int) round(yPos[i + j]));
                    update_dest(storedLength[j], i+j);
                }
            }
        }
    }
    
    void Simd_funcs::update_dest(float length, int n)
    {
        Ped::Twaypoint *waypoint = agents[n]->getNextDestinationSIMD(length); 
        if (waypoint != NULL)
        {
            xDest[n] = (float) waypoint->getx();
            yDest[n] = (float) waypoint->gety();
        }
    }
} 