rock_library(exploration
    SOURCES Planner.cpp
    HEADERS Planner.hpp ExplorationPlannerTypes.hpp Config.hpp
    DEPS_CMAKE OpenCV
    DEPS_PKGCONFIG base-types envire)

