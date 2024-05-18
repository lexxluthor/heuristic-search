#ifndef MISSION_H
#define	MISSION_H

#include <algorithm>
#include "map.h"
#include "agent_set.h"
#include "conflict_based_search.h"
#include "config.h"
#include "isearch.h"
#include "ilogger.h"
#include "searchresult.h"
#include "astar.h"
#include "focalsearch.h"
#include "xmllogger.h"
#include "multiagent_search_interface.h"
#include "testing_results.h"

//That's the wrap up class that first creates all the needed objects (Map, Search etc.)
//and then runs the search and then cleans everything up.

//Hint: Create Mission object in the main() function and then use it 1) to retreive all the data from input XML
//2) run the search 3) flush the results to output XML

class Mission
{
    public:
        Mission();
        Mission (const char* MapFile);
        ~Mission();

        bool getMap();
        bool getAgents(const char* agentsFile);
        bool getConfig();
        bool createLog();
        void createAlgorithm();
        void createEnvironmentOptions();
        bool checkAgentsCorrectness(const std::string &agentsFile);
        void startSearch(const std::string &agentsFile);
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();
        void saveAgentsPathsToLog(const std::string &agentsFile, double time, double makespan, double flowtime,
                                  double SIC, int HLExpansions, int HLNodes, int HLExpansionsStart, 
                                  int HLNodesStart, double LLExpansions, double LLNodes);
        bool checkCorrectness();
        void saveAggregatedResultsToLog();
        void saveSeparateResultsToLog();
        std::pair<int, int> getCosts();
        int getSIC(); 
        int getFirstTask() const;
        int getTasksCount() const;
        std::string getAgentsFile() const;
        bool getSingleExecution() const;
        bool getSaveAggregatedResults() const;

    private:
        Map                                   map;
        AgentSet                              agentSet;
        Config                                config;
        int                                   searchType;
        MultiagentSearchInterface*            multiagentSearch;
        ILogger*                              logger;
        const char*                           mapFile;
        MultiagentSearchResult                sr;
        std::vector<std::vector<Node>>        agentsPaths;
        std::vector<TestingResults>           testingResults;
        //std::map<int, std::vector<int>>       makespans;
        //std::map<int, std::vector<int>>       flowtimes;
        //std::map<int, std::vector<double>>    times;
};

#endif

