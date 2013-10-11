#include <map>

#include "Planner.hpp"
#include <stdio.h>

using namespace exploration;

typedef std::multimap<int,GridPoint> Queue;
typedef std::pair<int,GridPoint> Entry;


Planner::Planner()
{
	mFrontierCellCount = 0;
}

Planner::~Planner()
{
}

PointList Planner::getNeighbors(GridPoint p, bool diagonal)
{
	PointList neighbors;
	GridPoint n;
	
	n.x = p.x-1 ; n.y = p.y   ; neighbors.push_back(n);
	n.x = p.x   ; n.y = p.y-1 ; neighbors.push_back(n);
	n.x = p.x+1 ; n.y = p.y   ; neighbors.push_back(n);
	n.x = p.x   ; n.y = p.y+1 ; neighbors.push_back(n);
	
	if(diagonal)
	{
		n.x = p.x-1 ; n.y = p.y-1 ; neighbors.push_back(n);
		n.x = p.x-1 ; n.y = p.y+1 ; neighbors.push_back(n);
		n.x = p.x+1 ; n.y = p.y-1 ; neighbors.push_back(n);
		n.x = p.x+1 ; n.y = p.y+1 ; neighbors.push_back(n);
	}
	return neighbors;
}

PointList Planner::getFrontierCells(GridMap* map, GridPoint start, bool stopAtFirst)
{
	// Initialization
	mFrontierCellCount = 0;
	GridMap plan = GridMap(map->getWidth(), map->getHeight());
	PointList result;
	
	// Initialize the queue with the robot position
	Queue queue;
	queue.insert(Entry(0, start));
	plan.setData(start, 0);
	
	Queue::iterator next;
	int distance;
	GridPoint point;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		point = next->second;
		queue.erase(next);
		bool foundFrontier = false;
		
		// Add all adjacent cells
		PointList neighbors = getNeighbors(point);
		for(PointList::iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{
			if(map->getData(*cell) == -1)
			{
				foundFrontier = true;
				continue;
			}

			if(map->getData(*cell) == 0 && plan.getData(*cell) == -1)
			{
				queue.insert(Entry(distance+1, *cell));
				plan.setData(*cell,0);
			}
		}

		if(foundFrontier) 
		{
			GridPoint frontier = point;
			frontier.distance = distance;
			result.push_back(frontier);
			mFrontierCellCount++;
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

FrontierList Planner::getFrontiers(GridMap* map, GridPoint start)
{
	// Initialization
	mFrontierCellCount = 0;
	GridMap plan = GridMap(map->getWidth(), map->getHeight());
	FrontierList result;
	
	// Initialize the queue with the robot position
	Queue queue;
	queue.insert(Entry(0, start));
	plan.setData(start, 0);
	
	Queue::iterator next;
	int distance;
	GridPoint point;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{		
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		point = next->second;
		queue.erase(next);
		
		// Add neighbors
		bool isFrontier = false;
		PointList neighbors = getNeighbors(point, true);
		for(PointList::iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{
			if(map->getData(*cell) == -1)
			{
				isFrontier = true;
				continue;
			}
			if(map->getData(*cell) == 0 && plan.getData(*cell) == -1)
			{
				queue.insert(Entry(distance+1, *cell));
				plan.setData(*cell,0);
			}
		}
		
		if(isFrontier)
		{
			result.push_back(getFrontier(map, &plan, point));
		}
	}
	
	return result;
}

PointList Planner::getFrontier(GridMap* map, GridMap* plan, GridPoint start)
{
	PointList frontier;
	frontier.push_back(start);
	mFrontierCellCount++;
	
	return frontier;
}