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

		enum Status {SUCCESS, NO_GOAL, ERROR};
		
		/** Returns the result of the last operation */
		Status getStatus() {return mStatus;}
		
		/** Returns the result message of the last operation */
		char* getStatusMessage() {return mStatusMessage;}
		
		/** Returns a list of reachable frontier cells ordered by distance to 'start' */
		PointList getFrontierCells(GridMap* map, GridPoint start, bool stopAtFirst = false);
		
		/** Returns a set of connected frontiers ordered by distance to 'start' */
		FrontierList getFrontiers(GridMap* map, GridPoint start);
		
	private:
		Status mStatus;
		char mStatusMessage[500];
		
	};
}

#endif
