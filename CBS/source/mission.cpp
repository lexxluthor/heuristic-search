#include <iostream>

#include "mission.h"
#include "gl_const.h"


Mission::Mission()
{
    logger = nullptr;
    multiagentSearch = nullptr;
    mapFile = nullptr;
}

Mission::Mission (const char* MapFile)
{
    mapFile = MapFile;
    logger = nullptr;
    multiagentSearch = nullptr;
}

Mission::~Mission()
{
    delete logger;
    delete multiagentSearch;
}

bool Mission::getMap()
{
    return map.getMap(mapFile);
}

bool Mission::getAgents(const char* agentsFile)
{
    agentSet.clear();
    return agentSet.readAgents(agentsFile);
}

bool Mission::getConfig()
{
    return config.getConfig(mapFile);
}

bool Mission::createLog()
{
    delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(mapFile, config.LogParams);
}

void Mission::createAlgorithm()
{
    if (config.searchType == CN_ST_CBS) {
        if (config.lowLevel == CN_SP_ST_ASTAR) {
            multiagentSearch = new ConflictBasedSearch<Astar<>>(new Astar<>(true));
        } else if (config.lowLevel == CN_SP_ST_FS) {
            multiagentSearch = new ConflictBasedSearch<FocalSearch<>>(new FocalSearch<>(true, config.focalW));
        } 
    }
}

bool Mission::checkAgentsCorrectness(const std::string &agentsFile) {
    if (config.maxAgents != -1 && agentSet.getAgentCount() < config.maxAgents) {
        std::cout << "Warning: not enough agents in " << agentsFile <<
                     " agents file. This file will be ignored" << std::endl;
        return false;
    }
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        Agent agent = agentSet.getAgent(i);
        Node start = agent.getStartPosition(), goal = agent.getGoalPosition();
        if (!map.CellOnGrid(start.i, start.j) || !map.CellOnGrid(goal.i, goal.j) ||
            map.CellIsObstacle(start.i, start.j) || map.CellIsObstacle(goal.i, goal.j)) {
            std::cout << "Warning: start or goal position of agent " << agent.getId() << " in " << agentsFile <<
                         " agents file is incorrect. This file will be ignored" << std::endl;
            return false;
        }
    }
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        for (int j = i + 1; j < agentSet.getAgentCount(); ++j) {
            if (agentSet.getAgent(i).getStartPosition() == agentSet.getAgent(j).getStartPosition()) {
                std::cout << "Warning: start positions of agents " << i << " and " << j <<
                             " in " << agentsFile << " are in the same cell. This file will be ignored" << std::endl;
                return false;
            } else if (agentSet.getAgent(i).getGoalPosition() == agentSet.getAgent(j).getGoalPosition()) {
                std::cout << "Warning: goal positions of agents " << i << " and " << j <<
                             " in " << agentsFile << " are in the same cell. This file will be ignored" << std::endl;
                return false;
            }
        }
    }
    return true;
}

void Mission::startSearch(const std::string &agentsFile)
{
    int minAgents = config.singleExecution ? config.maxAgents : config.minAgents;
    int maxAgents = config.maxAgents == -1 ? agentSet.getAgentCount() : config.maxAgents;
    TestingResults res;
    for (int i = minAgents; i <= maxAgents; i += config.agentsStep) {
        AgentSet curAgentSet;
        for (int j = 0; j < i; ++j) {
            Agent agent = agentSet.getAgent(j);
            curAgentSet.addAgent(agent.getCur_i(), agent.getCur_j(), agent.getGoal_i(), agent.getGoal_j());
        }

        multiagentSearch->clear();

        std::chrono::steady_clock::time_point chrono_begin = std::chrono::steady_clock::now();
        std::clock_t c_start = std::clock();

        sr = multiagentSearch->startSearch(map, config, curAgentSet);

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        std::cout << "Chrono time: " << std::chrono::duration_cast<std::chrono::milliseconds>(now - chrono_begin).count() << std::endl;

        std::clock_t c_end = std::clock();
        std::cout << "Clock time: " << 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC << std::endl;

        if (!sr.pathfound) {
            std::cout << "Failed to find solution for " << i << " agents" << std::endl;
            if (config.singleExecution) {
                std::cout << "Log will not be created" << std::endl;
            }
            break;
        }

        agentsPaths = *(sr.agentsPaths);

        res.data[CNS_TAG_ATTR_MAKESPAN][i] = sr.makespan;
        res.data[CNS_TAG_ATTR_FLOWTIME][i] = sr.flowtime;
        res.data[CNS_TAG_ATTR_SIC][i] = sr.SIC;
        res.data[CNS_TAG_ATTR_TIME][i] = sr.time;
        res.data[CNS_TAG_ATTR_HLE][i] = sr.HLExpansions;
        res.data[CNS_TAG_ATTR_HLN][i] = sr.HLNodes;
        res.data[CNS_TAG_ATTR_HLES][i] = sr.HLExpansionsStart;
        res.data[CNS_TAG_ATTR_HLN][i] = sr.HLNodes;
        res.data[CNS_TAG_ATTR_HLNS][i] = sr.HLNodesStart;
        res.data[CNS_TAG_ATTR_LLE][i] = sr.AvgLLExpansions;
        res.data[CNS_TAG_ATTR_LLN][i] = sr.AvgLLNodes;
        res.data[CNS_TAG_FOCAL_W][i] = sr.focalW;
        res.data[CNS_TAG_ATTR_TN][i] = sr.totalNodes;
        res.finalTotalNodes[i] = sr.finalTotalNodes;
        res.finalHLNodes[i] = sr.finalHLNodes;
        res.finalHLNodesStart[i] = sr.finalHLNodesStart;
        res.finalHLExpansions[i] = sr.finalHLExpansions;
        res.finalHLExpansionsStart[i] = sr.finalHLExpansionsStart;

        if (config.singleExecution) {
            saveAgentsPathsToLog(agentsFile, sr.time.back(), sr.makespan.back(), sr.flowtime.back(),
                                 sr.SIC.back(), sr.HLExpansions.back(), sr.HLNodes.back(),
                                 sr.HLExpansionsStart.back(), sr.HLNodesStart.back(),
                                 sr.AvgLLExpansions.back(), sr.AvgLLNodes.back());
        }
        if (!checkCorrectness()) {
            std::cout << "Search returned incorrect results!" << std::endl;
            break;
        }
        std::cout << "Found solution for " << i << " agents. Time: " <<
                    sr.time.back() << ", flowtime: " << sr.flowtime.back() << 
                    ", makespan: " << sr.makespan.back() << ", SIC: " << sr.SIC.back() << std::endl;
    }
    testingResults.push_back(res);
}

std::pair<int, int> Mission::getCosts() {
    size_t makespan = 0, timeflow = 0;
    for (auto & agentsPath : agentsPaths) {
        makespan = std::max(makespan, agentsPath.size() - 1);
        int lastMove;
        for (lastMove = agentsPath.size() - 1; lastMove > 1 && agentsPath[lastMove] == agentsPath[lastMove - 1]; --lastMove);
        timeflow += lastMove;
    }
    return std::make_pair(makespan, timeflow);
}

int Mission::getSIC() {
    size_t SIC = 0;
    for (auto & agentsPath : agentsPaths)
        SIC += agentsPath.size() - 1;
    return SIC;
}

bool Mission::checkCorrectness() {
    size_t agentCount = agentsPaths.size();
    size_t solutionSize = 0;
    for (int j = 0; j < agentCount; ++j) {
        solutionSize = std::max(solutionSize, agentsPaths[j].size());
    }
    std::vector<std::vector<Node>::iterator> starts, ends;
    for (int j = 0; j < agentCount; ++j) {
        if (agentsPaths[j][0] != agentSet.getAgent(j).getStartPosition()) {
            std::cout << "Incorrect result: agent path starts in wrong position!" << std::endl;
            return false;
        }
        if (agentsPaths[j].back() != agentSet.getAgent(j).getGoalPosition()) {
            std::cout << "Incorrect result: agent path ends in wrong position!" << std::endl;
            return false;
        }
        starts.push_back(agentsPaths[j].begin());
        ends.push_back(agentsPaths[j].end());
    }

    for (int i = 0; i < solutionSize; ++i) {
        for (int j = 0; j < agentCount; ++j) {
            if (i >= agentsPaths[j].size()) {
                continue;
            }
            if (map.CellIsObstacle(agentsPaths[j][i].i, agentsPaths[j][i].j)) {
                std::cout << "Incorrect result: agent path goes through obstacle!" << std::endl;
                return false;
            }
            if (i > 0 &&
                abs(agentsPaths[j][i].i - agentsPaths[j][i - 1].i) +
                abs(agentsPaths[j][i].j - agentsPaths[j][i - 1].j) > 1) {
                std::cout << "Incorrect result: consecutive nodes in agent path are not adjacent!" << std::endl;
                return false;
            }
        }
    }
    ConflictSet conflictSet = ConflictBasedSearch<>::findConflict<std::vector<Node>::iterator>(starts, ends);
    if (!conflictSet.empty()) {
        Conflict conflict = conflictSet.getBestConflict();
        if (conflict.edgeConflict) {
            std::cout << "Incorrect result: two agents swap positions!" << std::endl;
        } else {
            std::cout << "Incorrect result: two agents occupy the same node!" << std::endl;
        }
        return false;
    }
    return true;
}

void Mission::saveSeparateResultsToLog() {
    for (int i = 0; i < testingResults.size(); ++i) {
        std::map<int, int> successCount;
        for (const auto& pair : testingResults[i].data[CNS_TAG_ATTR_TIME]) {
            successCount[pair.first] = 1;
        }
        logger->writeToLogAggregatedResults(successCount, testingResults[i],
                                            config.agentsFile + "-" + std::to_string(i + 1));
        logger->saveLog();
    }
}

void Mission::saveAggregatedResultsToLog() {
    std::map<int, int> successCounts;
    TestingResults aggRes;
    
    std::vector<std::string> keys = testingResults[0].getKeys();
    for (int i = config.minAgents; i <= config.maxAgents; ++i) {
        std::map<std::string, double> sums;
        for (const auto& key : keys) {
            sums[key] = 0.0;
        }
        int successCount = 0;
        for (auto res : testingResults) {
            if (res.data[CNS_TAG_ATTR_TIME].find(i) != res.data[CNS_TAG_ATTR_TIME].end()) {
                for (const auto& key : keys) {
                    sums[key] += res.data[key][i].back();
                }
                ++successCount;
            }
        }
        if (successCount == 0) {
            break;
        }
        successCounts[i] = successCount;
        for (const auto& key : keys) {
            aggRes.data[key][i] = {sums[key] / successCount};
        }
    }
    logger->writeToLogAggregatedResults(successCounts, aggRes);
    logger->saveLog();
}

void Mission::saveAgentsPathsToLog(const std::string &agentsFile, double time,
                                   double makespan, double flowtime, double SIC,
                                   int HLExpansions, int HLNodes,
                                   int HLExpansionsStart, int HLNodesStart,
                                   double LLExpansions, double LLNodes) {
    logger->writeToLogAgentsPaths(agentSet, agentsPaths, agentsFile, time, makespan, flowtime, SIC,
                                  HLExpansions, HLNodes, HLExpansionsStart, HLNodesStart, LLExpansions, LLNodes);
    logger->saveLog();
}

int Mission::getTasksCount() const {
    return config.tasksCount;
}

int Mission::getFirstTask() const {
    return config.firstTask;
}

std::string Mission::getAgentsFile() const {
    return config.agentsFile;
}

bool Mission::getSingleExecution() const {
    return config.singleExecution;
}

bool Mission::getSaveAggregatedResults() const {
    return config.saveAggregatedResults;
}
