#include <map>

#include "Planner.hpp"
#include <stdio.h>

using namespace exploration;

typedef std::multimap<int,GridPoint> Queue;
typedef std::pair<int,GridPoint> Entry;


Planner::Planner()
{
}

Planner::~Planner()
{
}

PointList Planner::getFrontierCells(GridMap* map, GridPoint start, bool stopAtFirst)
{
	// Create the result list
	PointList result;
	
	// Create some workspace for the wavefront algorithm
	GridMap plan = GridMap(map->getWidth(), map->getHeight());
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0, start);
	queue.insert(startPoint);
	plan.setData(start, 0);
	
	Queue::iterator next;
	int distance;
	GridPoint point;
	int cellCount = 0;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		point = next->second;
		queue.erase(next);
		bool foundFrontier = false;
		
		// Add all adjacent cells
		if(point.x <= 1 || point.x >= (map->getWidth() - 1) || point.y <= 1 || point.y >= (map->getHeight() -1))
		{
			// We reached the border of the map, which is unexplored terrain as well:
			foundFrontier = true;
		}else
		{
			GridPoint neighbors[4];
			neighbors[0] = neighbors[1] = neighbors[2] = neighbors[3] = point;
			neighbors[0].x -= 1;  // left
			neighbors[1].x += 1;  // right
			neighbors[2].y -= 1;  // up
			neighbors[3].y += 1;  // down
			
			for(unsigned int it = 0; it < 4; it++)
			{
				char mapValue, planValue;
				if(!map->getData(neighbors[it], mapValue))
				{
					mStatus = ERROR;
					sprintf(mStatusMessage, "Out of range error when accessing map.");
					return result;
				}
				if(!plan.getData(neighbors[it], planValue))
				{
					mStatus = ERROR;
					sprintf(mStatusMessage, "Out of range error when accessing plan.");
					return result;
				}

				if(mapValue == -1)
				{
					foundFrontier = true;
					break;
				}
				if(mapValue == 0 && planValue == -1)
				{
					queue.insert(Entry(distance+1, neighbors[it]));
					plan.setData(neighbors[it],distance+1);
				}
			}
		}
		if(foundFrontier) 
		{
			GridPoint frontier = point;
			frontier.distance = distance;
			result.push_back(frontier);
			if(stopAtFirst) break;
		}
	}
	
	// Set result message and return the point list
	if(result.size() > 0)
	{
		mStatus = SUCCESS;
		sprintf(mStatusMessage, "Found %d reachable frontier cells.", result.size());
	}else
	{
		mStatus = NO_GOAL;
		sprintf(mStatusMessage, "No reachable frontier cells found.");
	}
	return result;
}
