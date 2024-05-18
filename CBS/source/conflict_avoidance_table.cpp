#include "conflict_avoidance_table.h"

void ConflictAvoidanceTable::addNode(const Node &node) {
    auto tuple = std::make_tuple(node.i, node.j, node.g);
    if (nodeAgentsCount.find(tuple) == nodeAgentsCount.end()) {
        nodeAgentsCount[tuple] = 1;
    } else {
        ++nodeAgentsCount[tuple];
    }
}

void ConflictAvoidanceTable::addGoalNode(const Node &node) {
    goalNodeAgentsCount[std::make_pair(node.i, node.j)] = node.g;
}

void ConflictAvoidanceTable::addEdge(const Node &node, const Node &prev) {
    auto tuple = std::make_tuple(prev.i, prev.j, node.i, node.j, node.g);
    if (edgeAgentsCount.find(tuple) == edgeAgentsCount.end()) {
        edgeAgentsCount[tuple] = 1;
    } else {
        ++edgeAgentsCount[tuple];
    }
}

void ConflictAvoidanceTable::addAgentPath(const std::list<Node>::const_iterator& start,
                                          const std::list<Node>::const_iterator& end) {
    for (auto it = start; it != end; ++it) {
        if (std::next(it) == end) {
            addGoalNode(*it);
        } else {
            addNode(*it);
        }
        if (it != start && *it != *std::prev(it)) {
            addEdge(*it, *std::prev(it));
        }
    }
}

void ConflictAvoidanceTable::removeNode(const Node &node) {
    auto tuple = std::make_tuple(node.i, node.j, node.g);
    if (nodeAgentsCount[tuple] == 1) {
        nodeAgentsCount.erase(tuple);
    } else {
        --nodeAgentsCount[tuple];
    }
}

void ConflictAvoidanceTable::removeGoalNode(const Node &node) {
    goalNodeAgentsCount.erase(std::make_pair(node.i, node.j));
}

void ConflictAvoidanceTable::removeEdge(const Node &node, const Node &prev) {
    auto tuple = std::make_tuple(prev.i, prev.j, node.i, node.j, node.g);
    if (edgeAgentsCount[tuple] == 1) {
        edgeAgentsCount.erase(tuple);
    } else {
        --edgeAgentsCount[tuple];
    }
}

void ConflictAvoidanceTable::removeAgentPath(const std::list<Node>::const_iterator& start,
                                             const std::list<Node>::const_iterator& end) {
    for (auto it = start; it != end; ++it) {
        if (std::next(it) == end) {
            removeGoalNode(*it);
        } else {
            removeNode(*it);
        }
        if (it != start && *it != *std::prev(it)) {
            removeEdge(*it, *std::prev(it));
        }
    }
}

int ConflictAvoidanceTable::getAgentsCount(const Node &node, const Node &prev) const {
    int res = 0;
    auto nodeTuple = std::make_tuple(node.i, node.j, node.g);
    if (nodeAgentsCount.find(nodeTuple) != nodeAgentsCount.end()) {
        res += nodeAgentsCount.at(nodeTuple);
    }
    auto edgeTuple = std::make_tuple(node.i, node.j, prev.i, prev.j, node.g);
    if (edgeAgentsCount.find(edgeTuple) != edgeAgentsCount.end()) {
        res += edgeAgentsCount.at(edgeTuple);
    }

    auto it = goalNodeAgentsCount.find(std::make_pair(node.i, node.j));
    if (it != goalNodeAgentsCount.end() && it->second <= node.g) {
        ++res;
    }
    return res;
}

int ConflictAvoidanceTable::getNodeAgentsCount(const Node &node) const {
    int res = 0;
    auto nodeTuple = std::make_tuple(node.i, node.j, node.g);
    if (nodeAgentsCount.find(nodeTuple) != nodeAgentsCount.end()) {
        res += nodeAgentsCount.at(nodeTuple);
    }
    auto it = goalNodeAgentsCount.find(std::make_pair(node.i, node.j));
    if (it != goalNodeAgentsCount.end() && it->second <= node.g) {
        ++res;
    }
    return res;
}

int ConflictAvoidanceTable::getEdgeAgentsCount(const Node &node, const Node &prev) const {
    auto edgeTuple = std::make_tuple(node.i, node.j, prev.i, prev.j, node.g);
    if (edgeAgentsCount.find(edgeTuple) != edgeAgentsCount.end()) {
        return edgeAgentsCount.at(edgeTuple);
    }
    return 0;
}

int ConflictAvoidanceTable::getFirstSoftConflict(const Node & node, int startTime, int endTime) const {
    auto nodeIt = nodeAgentsCount.lower_bound(std::make_tuple(node.i, node.j, startTime));
    if (nodeIt != nodeAgentsCount.end() && std::get<0>(nodeIt->first) == node.i
            && std::get<1>(nodeIt->first) == node.j && std::get<2>(nodeIt->first) <= endTime) {
        return std::get<2>(nodeIt->first);
    }
    return -1;
}

int ConflictAvoidanceTable::getFutureConflictsCount(const Node & node, int time) const {
    return 0;
    int res = 0;
    auto it = nodeAgentsCount.upper_bound(std::make_tuple(node.i, node.j, time));
    for (; it != nodeAgentsCount.end() && std::get<0>(it->first) == node.i
                                   && std::get<1>(it->first) == node.j; ++it) {
        res += it->second;
    }
    return res;
}

void ConflictAvoidanceTable::getSoftConflictIntervals(std::vector<std::pair<int, int>> &res,
                                                      const Node & node, const Node &prevNode,
                                                      int startTime, int endTime, bool binary) const {
    std::map<int, int> agentsCount;
    auto nodeIt = nodeAgentsCount.lower_bound(std::make_tuple(node.i, node.j, startTime));
    auto nodeEnd = nodeAgentsCount.upper_bound(std::make_tuple(node.i, node.j, endTime));
    for (nodeIt; nodeIt != nodeEnd; ++nodeIt) {
        agentsCount[std::get<2>(nodeIt->first)] = nodeIt->second;
    }

    int count = 0, prevTime = startTime - 1, beg = -1;
    for (auto & it : agentsCount) {
        int time = it.first;
        if (time > prevTime + 1 || count == 0 || (!binary && it.second != count)) {
            if (beg != -1) {
                res.emplace_back(beg, count);
            }
            if (time > prevTime + 1) {
                res.emplace_back(prevTime + 1, 0);
            }
            beg = time;
            count = it.second;
        }
        prevTime = time;
    }
    if (beg != -1) {
        res.emplace_back(beg, count);
    }
    if (prevTime < endTime) {
        res.emplace_back(prevTime + 1, 0);
    }

    auto it = goalNodeAgentsCount.find(std::make_pair(node.i, node.j));
    if (it != goalNodeAgentsCount.end()) {
        int i;
        for (i = 0; i < res.size() && res[i].first < it->second; ++i) {}

        if (i == res.size() || res[i].first > it->second) {
            if (i > 0) {
                res.insert(res.begin() + i, std::make_pair(it->second, res[i - 1].second + 1));
                ++i;
            }
        }
        for (i; i < res.size(); ++i) {
            res[i].second += 1;
        }
    }
}
