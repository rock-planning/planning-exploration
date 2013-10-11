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

bool Planner::isFrontierCell(GridMap* map, GridPoint point)
{
	if(map->getData(point) != 0)
		return false;
		
	PointList neighbors = getNeighbors(point, true);
	for(PointList::iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		if(map->getData(*cell) == -1)
			return true;

	return false;
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
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		Queue::iterator next = queue.begin();
		int distance = next->first;
		GridPoint point = next->second;
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
	mFrontierCount = 0;
	GridMap plan = GridMap(map->getWidth(), map->getHeight());
	FrontierList result;
	
	// Initialize the queue with the robot position
	Queue queue;
	queue.insert(Entry(0, start));
	plan.setData(start, 0);
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{		
		// Get the nearest cell from the queue
		Queue::iterator next = queue.begin();
		int distance = next->first;
		GridPoint point = next->second;
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
		
		if(mFrontierCount > 200) break;
	}
	
	// Set result message and return the point list
	if(result.size() > 0)
	{
		mStatus = SUCCESS;
		sprintf(mStatusMessage, "Found %d frontiers with %d frontier cells.", mFrontierCount, mFrontierCellCount);
	}else
	{
		mStatus = NO_GOAL;
		sprintf(mStatusMessage, "No reachable frontiers found.");
	}
	return result;
}

PointList Planner::getFrontier(GridMap* map, GridMap* plan, GridPoint start)
{
	PointList frontier;
	mFrontierCount++;
	
	// Initialize the queue with the first frontier cell
	Queue queue;
	queue.insert(Entry(0, start));
	plan->setData(start, mFrontierCount);
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{		
		// Get the nearest cell from the queue
		Queue::iterator next = queue.begin();
		int distance = next->first;
		GridPoint point = next->second;
		queue.erase(next);
		
		// Add it to the frontier
		frontier.push_back(point);
		mFrontierCellCount++;
		
		// Add all adjacent frontier cells to the queue
		PointList neighbors = getNeighbors(point, true);
		for(PointList::iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{
			if(plan->getData(*cell) != mFrontierCount && isFrontierCell(map, *cell))
			{
				queue.insert(Entry(distance+1, *cell));
				plan->setData(*cell, mFrontierCount);
			}
		}
	}
	
	return frontier;
}