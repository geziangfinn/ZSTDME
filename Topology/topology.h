#ifndef TOPOLOGY_H
#define TOPOLOGY_H
#include "global.h"
#include "objects.h"
#include "ctsdb.h"

class NngNode : public Point_2D // a node used for nearest graph algorithm
{
public:
    NngNode()
    {
        SetZero();
        loadCapacitance = 0.0;
    }
    NngNode(double _x,double _y, double _loadCapacitance)
    {
        x=_x;
        y=_y;
        loadCapacitance=_loadCapacitance;
    }
    double loadCapacitance;
};

struct NngNodePair
{
    int from;
    int to;
    double cost;
    NngNodePair(int _from, int _to, double _cost)
    {
        from = _from;
        to = _to;
        cost = _cost;
    }
};

class TreeTopology
{
public:
    TreeTopology()
    {
        init();
    }
    TreeTopology(CTSDB *_db)
    {
        init();
        db = _db;
    }

    TreeNode *root;
    int leafCount;
    int nodeCount;
    unordered_map<int, TreeNode *> idToTreeNodeMap;
    CTSDB *db;

    void init()
    {
        root = NULL;
        leafCount = 0;
        nodeCount = 0;
        idToTreeNodeMap.clear();
        db = NULL;
    }
    void buildTopoUsingNearestNeighborGraph();
    double nearestNeighborPaitCost(NngNode,NngNode);
    void readTopologyFromFile();
};
#endif