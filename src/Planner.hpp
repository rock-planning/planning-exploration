#ifndef EXPLORATION_EXPLORATION_PLANNER_H
#define EXPLORATION_EXPLORATION_PLANNER_H

#include <exploration/ExplorationPlannerTypes.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <envire/Core.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

#include "Config.hpp"

namespace exploration
{    
    /**
     * Used to store data for each exploration point. 
     * After all expl. points have been processed the overallValue
     * can be calculated which is used to sort the points.
     */
    struct ExplorationPoint {
        
        ExplorationPoint(base::samples::RigidBodyState expl_pose,
                unsigned int num_expl_cells,
                double ang_dist,
                double robot_point_dist,
                double worst_driveability,
                bool edge_found) : 
                explPose(expl_pose),
                numberOfExploredCells(num_expl_cells),
                angularDistance(ang_dist),
                robotPointDistance(robot_point_dist),
                worstDriveability(worst_driveability),
                edgeFound(edge_found),
                expl_value(0.0),
                ang_value(0.0),
                dist_value(0.0),
                driveability_value(0.0),
                edge_value(0.0) {
        }
            
        bool operator<(const ExplorationPoint& rhs) const
        {
            return overallValue < rhs.overallValue;
        }
        
        /**
         * Calculates the overall value of this exploratin point.
         * If max_explored_cells or max_robot_goal_dist is 0, the explored cells
         * or the the distance of the goal point is ignored.
         * The bigger the better.
         */
        double calculateOverallValue(Weights& weights, double max_explored_cells, double max_robot_goal_dist) {
            if(max_explored_cells != 0) {
                expl_value = weights.explCells * (numberOfExploredCells / max_explored_cells);
            }
            ang_value = weights.angDist * (1-(angularDistance / M_PI));
            if(max_robot_goal_dist != 0) {
                dist_value = weights.robotGoalDist * (1-(robotPointDistance / max_robot_goal_dist));
            }
            driveability_value = weights.driveability * worstDriveability;
            edge_value = edgeFound ? weights.edgeDetected : 0;
            overallValue = expl_value + ang_value + dist_value + driveability_value + edge_value;
            return overallValue;
        }
    
        double overallValue;
        base::samples::RigidBodyState explPose;
        unsigned int numberOfExploredCells;
        double angularDistance;
        double robotPointDistance;
        double worstDriveability;
        // Is set to true if an edge has been searched and found or if 
        // the point is close to the robot and no edge detection has been required.
        // Otherwise far away edge points would win against close points for 
        // which an edge detection has not been required.
        bool edgeFound;
        
        double expl_value;
        double ang_value;
        double dist_value;
        double driveability_value;
        double edge_value;
    };
    
	class Planner
	{
	public:
		Planner(Config config = Config());
		~Planner();
		
		/** Returns the result of the last operation */
		Status getStatus() const {return mStatus;}
		
		/** Returns the result message of the last operation */
		const char* getStatusMessage() const {return mStatusMessage;}
		
		/** Returns a list of reachable frontier cells ordered by distance to 'start' */
		PointList getFrontierCells(GridMap* map, GridPoint start, bool stopAtFirst = false);
		
		/** Returns a set of connected frontiers ordered by distance to 'start' */
		FrontierList getFrontiers(GridMap* map, GridPoint start);
		
		/** Returns the number of reachable frontier cells after detection */
		unsigned int getFrontierCellCount() const {return mFrontierCellCount;}
		
		/** Initializes a new coverage map from a given grid map
		 *  All cells marked 'free' in the map are marked 'uncovered' in the coverage map */
		void initCoverageMap(GridMap* map);
		
		/** Set a cell to a given value (used to add new obstacles)
		 *  -1 : uncovered
		 *   0 : covered
		 *   1 : obstacle */
		void setCoverageMap(PointList points, char value = 1);
		
		/** Cover all cells within sensor footprint from the given pose. Uses the 'willBeExplored' function */
		void addReading(Pose p);
                
		/** Get all cells that are still uncovered */
		PointList getUnexploredCells() const;
		
		/** 
         * Add a sensor footprint to the internal sensor field 
         * Polygon has to be defined in grid coordinates.
         */
		void addSensor(Polygon p);
		
		/** Coordinate-transgform the given polygon to pose */ 
		Polygon transformPolygon(Polygon polygon, Pose pose);
		
		/** Get reachable Frontiers in coverage map from given pose **/
		FrontierList getCoverageFrontiers(Pose start);
		
		/** (Experimental!) 
		 * Currently returns nearest frontier cell in coverage map */
		Pose getCoverageTarget(Pose start);
                
        const GridMap& getCoverageMap() const;

        void setMinGoalDistance(double value) {min_goal_distance = value;}

        double getMinGoalDistance() const {return min_goal_distance;}
        
        /**
         * Uses the exploration map to calculate the orientation of the goal point
         * which should be orientated orthogonally to the known-unknown-border.
         * \param goal_point Requires grid coordinates.
         * \return If the passed point does not lies close to one of the found edges false is returned.
         */
        bool calculateGoalOrientation(struct Pose goal_point, double& orientation, bool show_debug=false);

        /**
         * Get the point with least angular difference to robotpose. Uses compare-function for sorting.
         * If calculate_worst_driveability is set to true the robot length and with have to be specified as well.
         * In this case obstacle poses are ignored and the worst driveability is used
         * within the cost calculations.
         * \param robotPose Expects local coordinates in meter.
         */
        std::vector<base::samples::RigidBodyState> getCheapest(std::vector<base::Vector3d> &pts, 
                base::samples::RigidBodyState &robotPose,
                bool calculate_worst_driveability = false,
                double robot_length_x = 0,
                double robot_width_y = 0);

        /**contains extracted travGrid **/
        envire::TraversabilityGrid* mTraversability;

        /**copy of exploremap as TravGrid. Used if the GridMap needs to be resized, thus transformed**/
        envire::TraversabilityGrid* mCoverageMapAsTravGrid;

        /**takes a GridMap and turns it into a TraversabilityGrid with the attributes of the given TravGrid**/
        envire::TraversabilityGrid* coverageMapToTravGrid(const GridMap& mapToBeTranslated, envire::TraversabilityGrid& traversability);
        
        /**
         * Clears the complete map / sets everything to UNKNOWN.
         * Receiving the next trav map will reset all OBSTACLES again.
         */
        inline void clearCoverageMap() {
            mCoverageMap->clearData();
        }
       
	private:
        Config mConfig;
		PointList getNeighbors(GridPoint p, bool diagonal = false) const;
		PointList getFrontier(GridMap* map, GridMap* plan, GridPoint start);
		bool isFrontierCell(GridMap* map, GridPoint point) const;
		bool pointInPolygon(FloatPoint point, Polygon polygon) const;
		bool isVisible(FloatPoint point, Pose pose) const;
		SensorField transformSensorField(Pose pose);
                
        /** returns all cells within the sensor footprint from the given pose */
        PointList willBeExplored(Pose p);

		Status mStatus;
		char mStatusMessage[500];
		
		unsigned int mFrontierCellCount;
		char mFrontierCount;
		
		GridMap* mCoverageMap;
		SensorField mSensorField;
                
        double min_goal_distance;
        
        /**
         * Maps the passed angle to [0, 2*PI)
         */
        double map0to2pi(double angle_rad);
        
        /**
         * Max distant sensor polygon point in grid coordinates.
         */
        double mMaxDistantSensorPoint;
	};
}

#endif
