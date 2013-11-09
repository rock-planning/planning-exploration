#ifndef EXPLORATION_EXPLORATION_PLANNER_H
#define EXPLORATION_EXPLORATION_PLANNER_H

#include "Types.hpp"

namespace exploration
{
	class Planner
	{
	public:
		Planner();
		~Planner();

		enum Status {SUCCESS, NO_GOAL, ERROR, NOT_IMPLEMENTED};
		
		/** Returns the result of the last operation */
		Status getStatus() {return mStatus;}
		
		/** Returns the result message of the last operation */
		char* getStatusMessage() {return mStatusMessage;}
		
		/** Returns a list of reachable frontier cells ordered by distance to 'start' */
		PointList getFrontierCells(GridMap* map, GridPoint start, bool stopAtFirst = false);
		
		/** Returns a set of connected frontiers ordered by distance to 'start' */
		FrontierList getFrontiers(GridMap* map, GridPoint start);
		
		/** Returns the number of reachable frontier cells after detection */
		unsigned int getFrontierCellCount() {return mFrontierCellCount;}
		
		/** Initializes a new coverage map from a given grid map
		 *  All cells marked 'free' in the map are marked 'uncovered' in the coverage map */
		void initCoverageMap(GridMap* map);
		
		/** Set a cell to a given value (used to add new obstacles)
		 *  -1 : uncovered
		 *   0 : covered
		 *   1 : obstacle */
		void setCoverageMap(PointList points, char value = 1);
		
		/** Cover all cells within sensor footprint from the given pose */
		void addReading(Pose p);
		
		/** Get all cells that are still uncovered */
		PointList getUnexploredCells();
		
		/** Add a sensor footprint to the internal sensor field */
		void addSensor(Polygon p) {mSensorField.push_back(p);}
		
		/** Coordinate-transgform the given polygon to pose */ 
		Polygon transformPolygon(Polygon polygon, Pose pose);
		
		/** Get reachable Frontiers in coverage map from given pose **/
		FrontierList getCoverageFrontiers(Pose start);
		
		/** (Experimental!) 
		 * Currently returns nearest frontier cell in coverage map */
		Pose getCoverageTarget(Pose start);
                
                const GridMap& getCoverageMap() const;
		
	private:
		PointList getNeighbors(GridPoint p, bool diagonal = false);
		PointList getFrontier(GridMap* map, GridMap* plan, GridPoint start);
		bool isFrontierCell(GridMap* map, GridPoint point);
		bool pointInPolygon(FloatPoint point, Polygon polygon);
		bool isVisible(FloatPoint point, Pose pose);
		SensorField transformSensorField(Pose pose);
	
		Status mStatus;
		char mStatusMessage[500];
		
		unsigned int mFrontierCellCount;
		char mFrontierCount;
		
		GridMap* mCoverageMap;
		SensorField mSensorField;
	};
}

#endif
