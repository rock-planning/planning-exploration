#include "Planner.hpp"

#include <map>
#include <stdio.h>
#include <math.h>

using namespace exploration;

typedef std::multimap<int,GridPoint> Queue;
typedef std::pair<int,GridPoint> Entry;


Planner::Planner()
{
	mFrontierCellCount = 0;
	mCoverageMap = NULL;
}

Planner::~Planner()
{
	if(mCoverageMap) delete mCoverageMap;
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
		sprintf(mStatusMessage, "Found %d reachable frontier cells.", (int)result.size());
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
		
		if(mFrontierCount > 100) break;
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

void Planner::initCoverageMap(GridMap* map)
{
	if(mCoverageMap) delete mCoverageMap;
	
	unsigned int width = map->getWidth();
	unsigned int height = map->getHeight();
	unsigned int size = width * height;
	mCoverageMap = new GridMap(width, height);
	
	char* origin = map->getData();
	for(unsigned int i = 0; i < size; i++)
	{
		if(origin[i] == 0)
			mCoverageMap->setData(i, -1);
		else
			mCoverageMap->setData(i, 1);
	}
}

void Planner::setCoverageMap(PointList points, char value)
{
	PointList::iterator i;
	for(i = points.begin(); i < points.end(); i++)
	{
		mCoverageMap->setData(*i, value);
	}
}

void Planner::addReading(Pose p)
{
	// TODO: Transform SensorFields to Pose p
	SensorField transformedSF = transformSensorField(p);
	
	// Rasterize transformed SensorField
	SensorField::iterator sensor;
	Polygon::iterator point;
	FloatPoint min, max, current;
	for(sensor = transformedSF.begin(); sensor < transformedSF.end(); sensor++)
	{
		// Determine bounding box for efficiency
		min = max = *(sensor->begin());
		for(point = sensor->begin()+1; point < sensor->end(); point++)
		{
			if(point->x < min.x) min.x = point->x;
			if(point->x > max.x) max.x = point->x;
			if(point->y < min.y) min.y = point->y;
			if(point->y > max.y) max.y = point->y;
		}
		
		// Rasterize polygon
		for(int y = min.y; y <= max.y; y++)
		{
			for(int x = min.x; x <= max.x; x++)
			{
				current.x = x;
				current.y = y;
				GridPoint gp;
				gp.x = x;
				gp.y = y;
				if(	mCoverageMap->getData(gp) == -1 && 
					pointInPolygon(current, *sensor) && 
					isVisible(current, p))
				{
					mCoverageMap->setData(gp, 0);
				}
			}
		}
	}
}

// http://alienryderflex.com/polygon/
bool Planner::pointInPolygon(FloatPoint point, Polygon polygon)
{
	int   j = polygon.size() - 1;
	bool  oddNodes = false;

	for (unsigned int i = 0; i < polygon.size(); i++)
	{
		if ((polygon[i].y < point.y && polygon[j].y >= point.y) ||
			(polygon[j].y < point.y && polygon[i].y >= point.y))
		{
			if (polygon[i].x + (point.y-polygon[i].y) / (polygon[j].y-polygon[i].y) * (polygon[j].x-polygon[i].x) < point.x)
			{
				oddNodes = !oddNodes;
			}
		}
		j = i;
	}
	return oddNodes;
}

bool Planner::isVisible(FloatPoint point, Pose pose)
{
	double x = pose.x;
	double y = pose.y;
	
	double delta_x = point.x - pose.x;
	double delta_y = point.y - pose.y;
	double delta = sqrt((delta_x*delta_x) + (delta_y*delta_y));
	int step = delta;
	double step_x = delta_x / step;
	double step_y = delta_y / step;
	
	for(int i = 0; i <= step; i++)
	{
		GridPoint p;
		p.x = x;
		p.y = y;
		if(mCoverageMap->getData(p) == 1)
			return false;
		
		x += step_x;
		y += step_y;
	}
	return true;
}

SensorField Planner::transformSensorField(Pose pose)
{
	SensorField::iterator sensor;
	SensorField field;
	for(sensor = mSensorField.begin(); sensor < mSensorField.end(); sensor++)
	{
		field.push_back(transformPolygon(*sensor, pose));
	}
	return field;
}

Polygon Planner::transformPolygon(Polygon polygon, Pose pose)
{
	Polygon::iterator p;
	for(p = polygon.begin(); p < polygon.end(); p++)
	{
		double x_ =  (p->x * cos(-pose.theta)) + (p->y * sin(-pose.theta));
		double y_ = -(p->x * sin(-pose.theta)) + (p->y * cos(-pose.theta));
		p->x = pose.x + x_;
		p->y = pose.y + y_;
	}
	return polygon;
}

PointList Planner::getUnexploredCells()
{
	PointList result;
	GridPoint p;
	
	for(unsigned int y = 0; y < mCoverageMap->getHeight(); y++)
	{
		for(unsigned int x = 0; x < mCoverageMap->getWidth(); x++)
		{
			p.x = x;
			p.y = y;
			if(mCoverageMap->getData(p) == -1) 
				result.push_back(p);
		}
	}	
	return result;
}
