#include "Planner.hpp"

#include <map>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <algorithm> // sort
#include <boost/concept_check.hpp>
#include <base/Logging.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace exploration;

typedef std::multimap<int,GridPoint> Queue;
typedef std::pair<int,GridPoint> Entry;


Planner::Planner()
{
    mFrontierCellCount = 0;
    mFrontierCount = 0;
    mCoverageMap = nullptr;
    mTraversability = nullptr;
    min_goal_distance = 0;
    mMaxDistantSensorPoint = -1;
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
     // Mark the cell as "already added to a frontier" by setting 
     // the value in the plan to 2.
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

void Planner::addSensor(Polygon p) {
    mSensorField.push_back(p);
    
    // Find max distant polygon point of all added sensors.
    // Used to determine which type of orientation calculation shuld be used. 
    std::vector<FloatPoint>::iterator it = p.begin();
    double len;
    for(; it != p.end(); it++) {
        len = it->norm();
        std::cout << "len " << len << " x " << it->x << " y " << it->y << std::endl;
        if(len > mMaxDistantSensorPoint) {
            mMaxDistantSensorPoint = len;
        }
    }
    std::cout << "Max len " << len << std::endl;
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

bool Planner::calculateGoalOrientation(struct Pose goal_point, double& orientation, bool show_debug){
    int num_cells_radius = 12;
    bool ret = false;
    
    // Defines opencv image (area around the goal point).
    int min_x = goal_point.x - num_cells_radius;
    int max_x = goal_point.x + num_cells_radius;
    int min_y = goal_point.y - num_cells_radius;
    int max_y = goal_point.y + num_cells_radius;
    if(min_x < 0)
        min_x = 0;
    if(min_y < 0)
        min_y = 0;
    if(max_x > (int)mCoverageMap->getWidth())
        max_x = mCoverageMap->getWidth();
    if(max_y > (int)mCoverageMap->getHeight())
        max_y = mCoverageMap->getHeight();
    
    int width_cv = max_x-min_x;
    int height_cv = max_y-min_y;
    if(width_cv <= 0 || height_cv <= 0) {
        LOG_WARN("Opencv image size is too small (%d, %d), orientation 0 will be returned");
        return false;
    }
    
    // Calculate goal position within the OpenCV image.
    struct Pose goal_point_cv;
    goal_point_cv.x = goal_point.x - min_x;
    goal_point_cv.y = goal_point.y - min_y;
    LOG_DEBUG("Calculate orientation for exploration point (%4.2f, %4.2f), opencv pixel (%d, %d)",
        goal_point.x, goal_point.y, goal_point_cv.x, goal_point_cv.y);

    // Copy image from the exploration map to the opencv image.
    // TODO Treat OBSTACLEs as unknown?
    cv::Mat mat_src(height_cv, width_cv, CV_8UC1, cv::Scalar(0));
    char* coverage_map = mCoverageMap->getData();
    int c = 0;
    int x_cv = 0;
    for(int x = min_x; x < max_x; ++x, ++x_cv) {
        int y_cv = 0;
        for(int y = min_y; y < max_y; ++y, ++y_cv) {
            c = (int)coverage_map[y*mCoverageMap->getWidth()+x];
            if(c == VISIBLE || c == EXPLORED) {
                mat_src.at<uchar>(y_cv,x_cv) = 255;
            } else { // OBSTACLE or UNKNOWN
                mat_src.at<uchar>(y_cv,x_cv) = 0;
            }
        }
    }
    
    cv::Mat mat_canny, mat_canny_bgr;
    Canny(mat_src, mat_canny, 50, 200);
    std::vector<cv::Vec2f> lines;
    // Resolution: 1 px and 180/32 degree.
    HoughLines(mat_canny, lines, 1, CV_PI/32, 10);

    if(show_debug) {
        cvtColor(mat_canny, mat_canny_bgr, CV_GRAY2BGR);
    }
        
    // Examines the found edges, choose the best one if available.
    cv::Point lowest_dist_pt1, lowest_dist_pt2;
    double shortest_dist = std::numeric_limits<double>::max();
    double expl_point_orientation = 0;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        
        // Determine the exploration point which lies closest to the found edge.
        double dist_diff_px = fabs((goal_point_cv.x * a + goal_point_cv.y * b) - rho);
        if(dist_diff_px < shortest_dist) {
            lowest_dist_pt1 = pt1;
            lowest_dist_pt2 = pt2;
            shortest_dist = dist_diff_px;
            expl_point_orientation = theta;
        }
        if(show_debug) {
            line( mat_canny_bgr, pt1, pt2, cv::Scalar(0,0,64), 1, 8/*CV_AA*/);
        }
    }
    
    // Calculate correct orientation (theta or theta+M_PI) by counting black pixels.
    // So, the direction with more black pixels is the direction the exploration should look at.
    int num_pixels_to_check = 10;
    goal_point_cv.theta = expl_point_orientation;
    int count_theta = countBlackPixels(mat_src, goal_point_cv , num_pixels_to_check);
    goal_point_cv.theta = expl_point_orientation + M_PI;
    int count_theta_pi = countBlackPixels(mat_src, goal_point_cv , num_pixels_to_check);
    if(abs(count_theta - count_theta_pi) >= 4) { // Difference is big enough.
        if(count_theta_pi > count_theta) { // Use orientation with more black / unknown pixels.
            expl_point_orientation = expl_point_orientation + M_PI;
        }
        
        // Draw line closest to the exploration point.
        if(shortest_dist <= 2.0) {
            LOG_DEBUG("Found edge lies close enough to the exploration point (%4.2f pixels)", shortest_dist);
            if(show_debug) {
                line( mat_canny_bgr, lowest_dist_pt1, lowest_dist_pt2, cv::Scalar(0,0,200), 1, 8/*CV_AA*/);
            }
            orientation = expl_point_orientation;
            ret = true;
        }
        
        if(show_debug) {
            // Draw cross goal point.
            cv::Point pt_e1, pt_e2;
            pt_e1.x = goal_point_cv.x - 2;
            pt_e1.y = goal_point_cv.y;
            pt_e2.x = goal_point_cv.x + 2;
            pt_e2.y = goal_point_cv.y;
            line( mat_canny_bgr, pt_e1, pt_e2, cv::Scalar(0,128,0), 1, 8/*CV_AA*/);
            pt_e1.x = goal_point_cv.x;
            pt_e1.y = goal_point_cv.y - 2;
            pt_e2.x = goal_point_cv.x;
            pt_e2.y = goal_point_cv.y + 2;
            line( mat_canny_bgr, pt_e1, pt_e2, cv::Scalar(0,128,0), 1, 8/*CV_AA*/);
            
            // Display windows.
            cv::Size size(300,300);//the dst image size,e.g.100x100   
            cv::Mat mat_src_resize, mat_canny_resize;
            resize(mat_src, mat_src_resize, size);
            resize(mat_canny_bgr, mat_canny_resize, size);
            
            
            imshow("source", mat_src_resize);
            imshow("detected lines", mat_canny_resize);
            
            cv::waitKey();
        }
    }
    return ret;
}

std::vector<base::samples::RigidBodyState> Planner::getCheapest(std::vector<base::Vector3d> &pts, 
        base::samples::RigidBodyState &roboPose,
        bool calculate_worst_driveability,
        double robot_length_x,
        double robot_width_y) {
    
    if(calculate_worst_driveability) {
        assert(robot_length_x > 0 && robot_width_y > 0);
    }
    
    bool visualize_debug_infos = false; 
    std::vector<std::tuple<base::samples::RigidBodyState, double, double, double, double> > listToBeSorted;
    listToBeSorted.reserve(pts.size());
    std::vector<base::samples::RigidBodyState> goals;
    goals.reserve(pts.size());
      
    double yaw = map0to2pi(roboPose.getYaw());

    // The for-loop is used for calculating the angular differences
    // experimental: dividing the number of cells that will be explored 
    // at the given point by the angDifference
    int too_close_counter = 0;
    int no_new_cell_counter = 0;
    int touch_obstacle = 0;
    int unknown_terrain_class = 0;
    int outside_of_the_map = 0;
    //int touch_difficult_region = 0;
    size_t robo_pt_x = 0, robo_pt_y = 0;
    bool robot_pos_grid_available = false;
    if(mTraversability->toGrid(roboPose.position, robo_pt_x, robo_pt_y, mTraversability->getFrameNode())) {
        robot_pos_grid_available = true;
    } else {
        LOG_WARN("Robot position (%4.2f, %4.2f) lies outside of the grid (%d, %d)", 
            roboPose.position[0], roboPose.position[1], robo_pt_x, robo_pt_y);
    }
        
    for(std::vector<base::Vector3d>::const_iterator i = pts.begin(); i != pts.end(); ++i)
    {
    Pose givenPoint;
    // Transform point to grid since its necessary for willBeExplored.
    size_t expl_pt_x = 0, expl_pt_y = 0;
    // Turn Vector3d into a RigidBodyState that finally will be pushed into the list of results.
    base::samples::RigidBodyState goalBodyState;
    goalBodyState.position = *i;
    if(mTraversability->toGrid(goalBodyState.position, expl_pt_x, expl_pt_y, mTraversability->getFrameNode()))
    {     
        givenPoint.x = expl_pt_x; 
        givenPoint.y = expl_pt_y;
    } else { // Should not happen.
        outside_of_the_map++;
        continue;
    }
    
    // If the distance between the robot and the exploration point exceeds the sensor range
    // (simply a far-away-goal) we use the edge detection to calculate the orientation
    // of the goal pose, otherwise the orientation of the vector between robot and goal is used.
    bool use_calculate_goal_orientation = false;
    double dist_robo_goal_grid = 0.0;
    if(robot_pos_grid_available && mMaxDistantSensorPoint > 0) {     
        dist_robo_goal_grid = sqrt(pow((int)expl_pt_x - (int)robo_pt_x, 2) + 
                pow((int)expl_pt_y - (int)robo_pt_y, 2));
        if(dist_robo_goal_grid > mMaxDistantSensorPoint) {
            use_calculate_goal_orientation = true;
        }
    } 
        
    // Calculate angle of exploregoal-vector and map it to 0-2*pi radian.
    double rotationOfPoint = 0.0;
    // If the exploration point lies close to the robot or if no matching edge 
    // can be found the old orientation calculation is used.
    bool edge_found = calculateGoalOrientation(givenPoint, rotationOfPoint, visualize_debug_infos);
    if(!(use_calculate_goal_orientation && edge_found)) {
        rotationOfPoint = atan2(i->y() - roboPose.position.y(), i->x() - roboPose.position.x()); 
        LOG_DEBUG("Goal orientation has been calculated using the current robot position");
    } else {
        LOG_DEBUG("Goal orientation has been calculated using edge detection");
    }
    rotationOfPoint = map0to2pi(rotationOfPoint);
    givenPoint.theta = rotationOfPoint;
    
    // Calculate angular distance, will be [0,2*PI)
    double angularDistance = fabs(yaw-rotationOfPoint);
    
    // Calculate the distance between the robot and the goalPose. 
    double robotToPointDistance = (goalBodyState.position - 
            base::Vector3d(roboPose.position.x(), roboPose.position.y(), 0)).norm();
    // Goal poses which are too close to the robot are discarded.
    if(robotToPointDistance < min_goal_distance || robotToPointDistance == 0) {
        too_close_counter++;
        continue;
    }
    
    // Assign orientation to goal position.
    goalBodyState.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(givenPoint.theta, Eigen::Vector3d::UnitZ()));
    
    // Ignore ecploration point if it is no real exploration point.
    unsigned numberOfExploredCells = willBeExplored(givenPoint).size();
    if(numberOfExploredCells <= 0) {
        no_new_cell_counter++;
        continue;
    }

    // Requests the worst driveability at the goal pose using the width/length of the robot.
    // If the goal pose rectangle touches an obstacle the goal point is ignored.
    double worst_driveability = 1.0;
    if(calculate_worst_driveability) {
        base::Pose2D pose_local;
        pose_local.position = base::Position2D(goalBodyState.position[0], goalBodyState.position[1]);
        pose_local.orientation = goalBodyState.getYaw();
        try {
            worst_driveability = mTraversability->getWorstTraversabilityClassInRectangle
                    (pose_local , robot_length_x, robot_width_y).getDrivability();
        } catch (std::runtime_error &e) { // Unknown terrain class.
            LOG_ERROR("Unknown terrain class");
            unknown_terrain_class++;
            continue;
        }
        if(worst_driveability == 0.0) {
            touch_obstacle++;
            continue;
        }
    }
    
    // Final rating of goalPose. robotToPointDistance cannot be zero.
    // Larger values are preferred.
    // Lazy, very curious exploration.
    //printf("Pose (%4.2f, %4.2f, %4.2f): ");
    double combinedRating = numberOfExploredCells / ((angularDistance+1) * robotToPointDistance);
    // Regarding the worst driveability as well creates a lazy, curious and cautious exploration.
    combinedRating *= worst_driveability;
    
    // Add it to the list which will be sorted afterwards. 
    // 2nd, 3rd... entry is for "debugging".
    listToBeSorted.push_back(std::make_tuple(goalBodyState, combinedRating, 
            numberOfExploredCells, angularDistance, robotToPointDistance));

    }
    
    LOG_INFO("%d of %d exploration points are uses: %d touches an obstacle, %d are too close to the robot, %d leads to no new cells, %d lies outside of the map", 
        listToBeSorted.size(), pts.size(), touch_obstacle, too_close_counter, no_new_cell_counter, outside_of_the_map);
    
    // Sorting list by comparing combinedRating. uses the given lambda-function 
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
    } else if(visualize_debug_infos) {
        // Adds best goal to the intern grid map for visualization.
        base::samples::RigidBodyState best_rbs_local = *(goals.begin());
        base::Pose2D pose2d;
        pose2d.position = base::Vector2d(best_rbs_local.position[0], best_rbs_local.position[1]);
        pose2d.orientation = best_rbs_local.getYaw();
        mTraversability->forEachInRectangle(pose2d, robot_length_x, robot_width_y, [&] (size_t x, size_t y) {
            mCoverageMap->setData(GridPoint(x,y,0), GOAL_CELL);
        });
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
    exploreMap->setTraversabilityClass(GOAL_CELL, envire::TraversabilityClass (0.75)); //goal rectangle
    
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
                case GOAL_CELL:
                    exp_array[y][x] = GOAL_CELL;
                    break;
                default:
                    break;
            }
        }
    }
    return exploreMap;
}

// PRIVATE
double Planner::map0to2pi(double angle_rad) {
    double pi2 = 2*M_PI;
    while(angle_rad >= pi2) {
        angle_rad -= pi2;
    }
    while(angle_rad < 0) {
        angle_rad += pi2;
    }
    return angle_rad;
}

int Planner::countBlackPixels(cv::Mat mat, struct Pose pose, int vec_len_px) {
    base::Vector2d start(pose.x, pose.y);
    double orientation = pose.theta;
    int count_black = 0;
    
    for(int i=1; i<=vec_len_px; i++) {
        base::Vector2d vec(i, 0.0);
        
        Eigen::Rotation2D<double> rot2(orientation);
        vec = rot2 * vec;
        vec += start;
        if(vec[0] < 0 || vec[0] >= mat.cols ||
            vec[1] < 0 || vec[1] >= mat.rows) {
            return count_black;
        }
        if(mat.at<uchar>(vec[1], vec[0]) == 0) {
            count_black++;
        }
    }
    return count_black;
}