#ifndef MDD_H
#define MDD_H

#include <vector>
#include <unordered_set>
#include "constraints_set.h"
#include "map.h"
#include "agent_set.h"
#include "isearch.h"
#include "astar.h"

class MDD
{
public:
    MDD();
    template<typename SearchType>
    MDD(const Map& map, const AgentSet& agentSet, SearchType* search, int agentId, int cost,
        const ConstraintsSet& constraints = ConstraintsSet());

    int getLayerSize(int cost) const;

//private:
    std::vector<int> layerSizes;
};

template <typename SearchType>
MDD::MDD(const Map& map, const AgentSet& agentSet, SearchType* search, int agentId, int cost, const ConstraintsSet& constraints) {
    Astar<> astar;
    Agent agent = agentSet.getAgent(agentId);
    Node start = agent.getStartPosition(), goal = agent.getGoalPosition();
    std::vector<std::unordered_set<Node>> layers;
    layers.push_back({start});

    int t = 0;
    int q = 0;

    for (int i = 0; i < cost - 1; ++i) {
        layers.push_back({});
        for (auto node : layers[i]) {

            std::chrono::steady_clock::time_point a = std::chrono::steady_clock::now();

            std::list<Node> successors = astar.findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);

            std::chrono::steady_clock::time_point b = std::chrono::steady_clock::now();
            q += std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
            ++t;

            for (auto neigh : successors) {
                if (search->computeHFromCellToCell(neigh.i, neigh.j, goal.i, goal.j) <= cost - i - 1) {
                    layers.back().insert(neigh);
                }
            }
        }
    }

    layerSizes.resize(cost + 1, 0);
    layerSizes[cost] = 1;
    std::unordered_set<Node> lastLayer = {goal};
    for (int i = cost - 1; i >= 0; --i) {
        std::unordered_set<Node> newLastLayer;
        for (auto node : layers[i]) {
            std::list<Node> successors = astar.findSuccessors(node, map, goal.i, goal.j, agentId, {}, constraints);
            for (auto neigh : successors) {
                if (lastLayer.find(neigh) != lastLayer.end()) {
                    newLastLayer.insert(node);
                    break;
                }
            }
        }
        layerSizes[i] = newLastLayer.size();
        lastLayer = newLastLayer;
    }
}

#endif // MDD_H
