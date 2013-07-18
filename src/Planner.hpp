#ifndef _EXPLORATION_PLANNER_HPP_
#define _EXPLORATION_PLANNER_HPP_

#include <base/Pose.hpp>
#include <base/time.h>

namespace exploration
{
	class Planner
	{
	public: 
		Planner();
		~Planner();
			
		base::Pose2D getExplorationTarget();

	private:
			base::Time mTimeStamp;
			
	};

}

#endif
