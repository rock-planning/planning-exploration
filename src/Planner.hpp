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
		
		void initCoverageMap(GridMap* map);
		void setCoverageMap(PointList points, char value);
		void addReading(Pose p);
		PointList getUnexploredCells();
		void addSensor(Polygon p) {mSensorField.push_back(p);}
		Polygon transformPolygon(Polygon polygon, Pose pose);
		Pose getCoverageTarget(Pose start);
		
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
