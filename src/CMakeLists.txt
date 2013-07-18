rock_library(exploration
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(exploration_bin Main.cpp
    DEPS exploration)

