cmake_minimum_required(VERSION 2.8)
project(CBS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -Wall -Wextra")
include_directories(.)
set(SOURCE_FILES
        agent.cpp
        agent_set.cpp
        astar.cpp
        config.cpp
        conflict_avoidance_table.cpp
        conflict_based_search.cpp
        conflict_set.cpp
        constraints_set.cpp
        dijkstra.cpp
        focalsearch.cpp
        isearch.cpp
        map.cpp
        mdd.cpp
        mission.cpp
        search_queue.cpp
        tinyxml2.cpp
        xmllogger.cpp
        agent.h
        agent_move.h
        agent_set.h
        astar.h
        cbs_node.h
        config.h
        conflict_avoidance_table.h
        conflict_based_search.h
        conflict_set.h
        constraint.h
        constraints_set.h
        dijkstra.h
        focalsearch.h
        fs_node.h
        gl_const.h
        ilogger.h
        isearch.h
        map.h
        mdd.h
        mission.h
        multiagent_search_interface.h
        multiagent_search_result.h
        node.h
        search_queue.h
        searchresult.h
        testing_results.h
        tinyxml2.h
        xmllogger.h
        )
        
add_executable(${PROJECT_NAME} ${SOURCE_FILES} main.cpp)

