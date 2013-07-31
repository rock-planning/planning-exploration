#ifndef _EXPLORATION_PLANNER_HPP_
#define _EXPLORATION_PLANNER_HPP_

#include <base/Pose.hpp>
#include <envire/maps/TraversabilityGrid.hpp>

namespace exploration
{
	class Planner
	{
	public: 
		Planner();
		~Planner();
			
		base::Pose2D getExplorationTarget(envire::TraversabilityGrid map, base::Pose2D pose);

	private:
			
	};

}

#endif
