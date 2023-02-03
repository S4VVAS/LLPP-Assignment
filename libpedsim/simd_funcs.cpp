#include "simd_funcs.h"
#include <iostream>
#include<cmath>

namespace Ped
{
    Simd_funcs::Simd_funcs(std::vector<Ped::Tagent*> startAgents)
    {
        agents = startAgents;
        std::cout << "Creating\n";
        // Allocate floats with SSE-compatible alignment
        xPos  = (float*) _mm_malloc(agents.size() *sizeof(float), 16);
        yPos  = (float*) _mm_malloc(agents.size() *sizeof(float), 16);
        xDest = (float*) _mm_malloc(agents.size() *sizeof(float), 16);
        yDest = (float*) _mm_malloc(agents.size() *sizeof(float), 16);

       
	   // Set up the start-positions and start destinations
        for (int i = 0; i < agents.size(); i++)
        {
            xPos[i]  = (float) agents[i]->getX();
            yPos[i]  = (float) agents[i]->getY();
            // This is probably bad but it segfaults without initial positions
            agents[i]->computeNextDesiredPosition();
            xDest[i] = (float) agents[i]->getDestination()->getx();
            yDest[i] = (float) agents[i]->getDestination()->gety();
        }
	std::cout << "Created\n";
    }

  void Simd_funcs::update_pos()
    {
        for (int i = 0; i < agents.size(); i += 4)
        {
            // CHEAT: if there are less than 4 instructions on the last iteration, skip
            // until we figure out what to do here! We don't want a seg fault!
            if (agents.size() - i < 4)
                return;
	    std::cout << "UPDATING\n";

            // Set-up SSE-variables
            __m128 XPOS;
            __m128 YPOS;
            __m128 diffX;
            __m128 diffY;
            __m128 len; 
            __m128 t0;
            __m128 t1;

	    std::cout << "LOADING\n";
            // Load values into SSE
            XPOS = _mm_load_ps(&xPos[i]);
            YPOS = _mm_load_ps(&yPos[i]);
            t0 = _mm_load_ps(&xDest[i]);
            t1 = _mm_load_ps(&yDest[i]);

	    std::cout << "CALCULATING\n";
            // Calculate xDest - xPos (and same with y)
            diffX = _mm_sub_ps(t0, XPOS);
            diffY = _mm_sub_ps(t1, YPOS);
            // sqrt(diffX * diffX + diffY * diffY)
            t0 = _mm_mul_ps(diffX, diffX);
            t1 = _mm_mul_ps(diffY, diffY);
            len = _mm_add_ps(t0, t1);
            len = _mm_sqrt_ps(t0);
    
            // Check whether position is reached and if so update destination
            
            float storeX[4];
            _mm_store_ps(storeX, len); // store length in storeX
            update_dest(storeX[0], agents[i+0], i+0);
            update_dest(storeX[1], agents[i+1], i+1);
            update_dest(storeX[2], agents[i+2], i+2);
            update_dest(storeX[3], agents[i+3], i+3);
            
	    std::cout << "DESIRED\n";
            //desiredPositionX = (int)round(x + diffX / len);
            diffX = _mm_div_ps(diffX, len);
            XPOS  = _mm_add_ps(XPOS, diffX);
            //desiredPositionY = (int)round(y + diffY / len);
            diffY  = _mm_div_ps(t1, len);
            YPOS   = _mm_add_ps(YPOS, diffY);

	    std::cout << "STORING\n";
            // Store the values back
            _mm_store_ps(&xPos[i], XPOS);
            _mm_store_ps(&yPos[i], YPOS);

	    std::cout << "AGENTS\n";
            // Store x and y in agents - however this is bad and slow!
            agents[i + 0]->setX((int) floor(xPos[i + 0]));
            agents[i + 1]->setX((int) floor(xPos[i + 1]));
            agents[i + 2]->setX((int) floor(xPos[i + 2]));
            agents[i + 3]->setX((int) floor(xPos[i + 3]));
            agents[i + 0]->setY((int) floor(yPos[i + 0]));
            agents[i + 1]->setY((int) floor(yPos[i + 1]));
            agents[i + 2]->setY((int) floor(yPos[i + 2]));
            agents[i + 3]->setY((int) floor(yPos[i + 3]));
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
