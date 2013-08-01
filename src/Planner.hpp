#ifndef EXPLORATION_EXPLORATION_PLANNER_H
#define EXPLORATION_EXPLORATION_PLANNER_H

#include "Types.hpp"

#define PLANNER_SUCCESS 0
#define PLANNER_NO_GOAL 100
#define PLANNER_ERROR   400

namespace exploration
{
	class Planner
	{
	public:
		Planner();
		~Planner();

		int explore(GridMap map, GridPoint start, GridPoint &goal);
	};
}

#endif
