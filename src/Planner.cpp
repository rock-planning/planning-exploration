#include "Planner.hpp"

#include <map>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm> // sort
#include <boost/concept_check.hpp>
#include <base/Logging.hpp>

using namespace exploration;

typedef std::multimap<int,GridPoint> Queue;
typedef std::pair<int,GridPoint> Entry;


Planner::Planner()
{
    mFrontierCellCount = 0;
    mCoverageMap = nullptr;
    mTraversability = nullptr;
}

Planner::~Planner()
{
    if(mCoverageMap) 
        delete mCoverageMap;
    if(mTraversability) 
        delete mTraversability;
}

PointList Planner::getNeighbors(GridPoint p, bool diagonal) const
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

bool Planner::isFrontierCell(GridMap* map, GridPoint point) const
{
    char c = 0;
	if(map->getData(point, c) && c != VISIBLE)
		return false;
		
	PointList neighbors = getNeighbors(point, true);
	for(PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++) {
        char c = 0;
		if(map->getData(*cell, c) && c == UNKNOWN) { 
			return true;
        }
    }

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
		for(PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{
            char c = 0;
			if(map->getData(*cell, c) && c == UNKNOWN)
			{
				foundFrontier = true;
				continue;
			}

			if(map->getData(*cell, c) && c == VISIBLE && plan.getData(*cell, c) && c == UNKNOWN)
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
	plan.setData(start, VISIBLE);
	
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
		PointList neighbors = getNeighbors(point, false);
        char c = 0;
		for(PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{   
            if(map->getData(*cell,c) && c == UNKNOWN)
            {
                plan.setData(*cell, OBSTACLE);
                isFrontier = true;
                break;
            }
            if(map->getData(*cell, c) && c == VISIBLE && 
                plan.getData(*cell, c) && c == UNKNOWN)
            {
                queue.insert(Entry(distance+1, *cell));
                plan.setData(*cell, VISIBLE);
            }
		}
		
		if(isFrontier)
		{
			result.push_back(getFrontier(map, &plan, point));
		}
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
    /**
     * mark the cell as "already added to a frontier" by setting 
     * the value in the plan to 2
     */
	PointList frontier;
	mFrontierCount++;
	
	// Initialize the queue with the first frontier cell
	Queue queue;
	queue.insert(Entry(0, start));
	plan->setData(start, OBSTACLE);
	
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
        char c = 0;
		for(PointList::const_iterator cell = neighbors.begin(); cell < neighbors.end(); cell++)
		{
			if(plan->getData(*cell, c) && c != OBSTACLE && isFrontierCell(map, *cell))
			{
				plan->setData(*cell, OBSTACLE);
				queue.insert(Entry(distance+1, *cell));
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
	mCoverageMap = new GridMap(width, height);
	
    memcpy(mCoverageMap->getData(), map->getData(), sizeof(char)*width*height);
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
    PointList points = willBeExplored(p);
    for(PointList::iterator i = points.begin(); i < points.end(); ++i)
    {
        mCoverageMap->setData(*i, VISIBLE);
    }
}

PointList Planner::willBeExplored(Pose p)
{
    // TODO: Transform SensorFields to Pose p
    SensorField transformedSF = transformSensorField(p);
    
    // Rasterize transformed SensorField
    SensorField::iterator sensor;
    Polygon::iterator point;
    FloatPoint min, max, current;
    PointList result;
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
                char c = 0;
                if(     mCoverageMap->getData(gp, c) &&
                        c == UNKNOWN && 
                        pointInPolygon(current, *sensor) && 
                        isVisible(current, p))
                {
                        result.push_back(gp);
                }
            }
        }
    }
    return result;
}


// http://alienryderflex.com/polygon/
bool Planner::pointInPolygon(FloatPoint point, Polygon polygon) const
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

bool Planner::isVisible(FloatPoint point, Pose pose) const
{
	double x = pose.x;
	double y = pose.y;
	
	double delta_x = point.x - pose.x;
	double delta_y = point.y - pose.y;
	double delta = sqrt((delta_x*delta_x) + (delta_y*delta_y));
	int step = delta;
	double step_x = delta_x / step;
	double step_y = delta_y / step;
    char c = 0;
	
	for(int i = 0; i < step; i++)
	{
		GridPoint p;
		p.x = x;
		p.y = y;
		if(mCoverageMap->getData(p, c) && c == 1) // TODO What does 1 means?
			return false;
		
		x += step_x;
		y += step_y;
	}
	return true;
}

SensorField Planner::transformSensorField(Pose pose)
{
	SensorField::const_iterator sensor;
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

PointList Planner::getUnexploredCells() const
{
	PointList result;
	GridPoint p;
    char c = 0;
	
	for(unsigned int y = 0; y < mCoverageMap->getHeight(); y++)
	{
		for(unsigned int x = 0; x < mCoverageMap->getWidth(); x++)
		{
			p.x = x;
			p.y = y;
			if(mCoverageMap->getData(p, c) && c == UNKNOWN) {
				result.push_back(p);
            }
		}
	}	
	return result;
}

Pose Planner::getCoverageTarget(Pose start)
{
	GridPoint startPoint;
	startPoint.x = start.x;
	startPoint.y = start.y;
	PointList fCells = getFrontierCells(mCoverageMap, startPoint);
	PointList::const_iterator p;
	Pose target;
	for(p = fCells.begin(); p < fCells.end(); p++)
	{
		target.x = p->x;
		target.y = p->y;
                LOG_DEBUG_S << "possible goals: " << target.x << "/" << target.y;
		if(p->distance > 20 && p->distance < 30) break;
	}
	return target;
}

std::vector<base::samples::RigidBodyState> Planner::getCheapest(std::vector<base::Vector3d> &pts, base::samples::RigidBodyState &roboPose)
{
     double yaw;
     // If the robot-orientatin is negative it will be mapped to 2*pi 
    if(roboPose.getYaw() < 0)
    {
        yaw = 2*M_PI + roboPose.getYaw();
    } else { 
        yaw = roboPose.getYaw();
    }
    LOG_DEBUG_S << "yaw is: " << yaw;
    
     std::vector<std::tuple<base::samples::RigidBodyState, double, double, double, double> > listToBeSorted;
     listToBeSorted.reserve(pts.size());
     std::vector<base::samples::RigidBodyState> goals;
     goals.reserve(pts.size());
     
     // The for-loop is used for calculating the angular differences
     // experimental: dividing the number of cells that will be explored 
     // at the given point by the angDifference
     for(std::vector<base::Vector3d>::const_iterator i = pts.begin(); i != pts.end(); ++i)
     {
        // Calculate angle of exploregoal-vector and map it to 0-2*pi radian.
        double rotationOfPoint = atan2(i->y() - roboPose.position.y(), i->x() - roboPose.position.x()); 
        if(rotationOfPoint > 2*M_PI)
        {
            rotationOfPoint = fmod(rotationOfPoint, 2*M_PI);
        }
        if(rotationOfPoint < 0) 
        {
            rotationOfPoint = 2*M_PI+rotationOfPoint;
        } 
        // Calculate angular distance.
        double angularDistance = fabs(yaw-rotationOfPoint);
        if(angularDistance > M_PI)
        {
            angularDistance = 2*M_PI - angularDistance;
        }
        
        Pose givenPoint;
        // Turn Vector3d into a RigidBodyState that finally will be pushed into the list of results.
        base::samples::RigidBodyState goalBodyState;
        goalBodyState.position = *i;
        // Calculate the distance between the robot and the goalPose. necessary for evaluation of goalPose.
        double robotToPointDistance = (goalBodyState.position - base::Vector3d(roboPose.position.x(), roboPose.position.y(), 0)).norm();
        if(robotToPointDistance < min_goal_distance) continue;
        
        // Transform point to grid since its necessary for willBeExplored.
        size_t x, y;
        if(mTraversability->toGrid(goalBodyState.position, x, y, mTraversability->getFrameNode()))
        {     
            givenPoint.theta = rotationOfPoint;
            givenPoint.x = x; givenPoint.y = y;
        }
        
        goalBodyState.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(givenPoint.theta, Eigen::Vector3d::UnitZ()));
        
        unsigned numberOfExploredCells = willBeExplored(givenPoint).size();
        if(numberOfExploredCells > 0)
        {
             // Final rating of goalPose.
            double combinedRating = numberOfExploredCells / (angularDistance+1) / robotToPointDistance;
            
            // Add it to the list which will be sorted afterwards. 
            // 2nd, 3rd... entry is for "debugging".
            listToBeSorted.push_back(std::make_tuple(goalBodyState, combinedRating, 
                    numberOfExploredCells, angularDistance, robotToPointDistance));
        }
     }
     
     // Sorting list by comparing the angular differences. uses the given lambda-function 
     std::sort(listToBeSorted.begin(), listToBeSorted.end(), 
               [](const std::tuple<base::samples::RigidBodyState, double, double, double, double>& i, 
                  const std::tuple<base::samples::RigidBodyState, double, double, double, double>& j) { 
         return std::get<1>(i) > std::get<1>(j); 
     });
     
     // Copy the goalvectors of the sorted list of triples into the std::vector that is going to be dumped on the port.
     std::vector<std::tuple<base::samples::RigidBodyState, double, double,  double, double> >::const_iterator i;
     for(i = listToBeSorted.begin(); i != listToBeSorted.end(); ++i)
     {
         base::samples::RigidBodyState targetPose = std::get<0>(*i);

         targetPose.targetFrame = "world";
         targetPose.time = base::Time::now();
         LOG_DEBUG_S << "pushed point: " << targetPose.position.x() << "/" << targetPose.position.y() << 
                " with rating: " << std::get<1>(*i) << " . Cells: " << std::get<2>(*i) << 
                " . AngDistance: " << std::get<3>(*i) << " . Distance: " << std::get<4>(*i);
         goals.push_back(targetPose);
     }
     
     if(goals.empty())
     {
        LOG_WARN_S << "did not find any target, propably stuck in an obstacle.";
     } 
     
     return goals;
}

FrontierList Planner::getCoverageFrontiers(Pose start)
{
	GridPoint startPoint;
	startPoint.x = start.x;
	startPoint.y = start.y;
	return getFrontiers(mCoverageMap, startPoint);
}

const exploration::GridMap& Planner::getCoverageMap() const
{
    return *mCoverageMap;
}

envire::TraversabilityGrid* Planner::coverageMapToTravGrid(const GridMap& mapToBeTranslated, envire::TraversabilityGrid& traversability)
{
    // Initializing exploreMap that is going to be dumped.
    envire::TraversabilityGrid *exploreMap = new envire::TraversabilityGrid(
            traversability.getWidth(), traversability.getHeight(), 
            traversability.getScaleX(), traversability.getScaleY(), 
            traversability.getOffsetX(), traversability.getOffsetY());
    exploreMap->setTraversabilityClass(OBSTACLE, envire::TraversabilityClass (0.0)); //obstacle
    exploreMap->setTraversabilityClass(EXPLORED, envire::TraversabilityClass (1.0)); //explored
    exploreMap->setTraversabilityClass(UNKNOWN, envire::TraversabilityClass (0.5)); //unknown
    
    envire::TraversabilityGrid::ArrayType& exp_array = exploreMap->getGridData();
    
    const GridMap &map(mapToBeTranslated);
    
    size_t xiExplo = traversability.getCellSizeX();
    size_t yiExplo = traversability.getCellSizeY();

    struct GridPoint pointExplo;
    char value = 0;
    
    for(size_t y = 0; y < yiExplo; y++){
        pointExplo.y = y;
        for (size_t x = 0; x < xiExplo; x++){
            pointExplo.x = x;
            map.getData(pointExplo, value);
            exploreMap->setProbability(1.0, x, y);
            switch(value)
            {
                case UNKNOWN:
                    exp_array[y][x] = UNKNOWN;
                    break;
                case EXPLORED:
                    exp_array[y][x] = EXPLORED;
                    break;
                case OBSTACLE:
                    exp_array[y][x] = OBSTACLE;
                    break;
                default:
                    break;
            }
        }
    }
    return exploreMap;
}
