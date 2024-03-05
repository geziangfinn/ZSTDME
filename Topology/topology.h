#ifndef TOPOLOGY_H
#define TOPOLOGY_H
#include "global.h"
#include "objects.h"
#include "ctsdb.h"

class NNGGrid;

class NngNode : public Point_2D // a node used for nearest graph algorithm
{
public:
    NngNode()
    {
        SetZero();
        loadCapacitance = 0.0;
    }
    NngNode(double _x, double _y, double _loadCapacitance)
    {
        x = _x;
        y = _y;
        loadCapacitance = _loadCapacitance;
    }
    double loadCapacitance;
};

struct NngNodePair
{
    int from; // from and to are both index of the vector (global)treeNodes
    int to;
    double cost;
    NngNodePair(int _from, int _to, double _cost)
    {
        from = _from;
        to = _to;
        cost = _cost;
    }
    bool operator<(const NngNodePair &a) const { return cost < a.cost; }
    bool operator>(const NngNodePair &a) const { return cost > a.cost; }
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
        leafCount = db->dbSinks.size();
    }

    TreeNode *root;
    int leafCount;
    int nodeCount;
    unordered_map<int, TreeNode *> idToTreeNodeMap;
    CTSDB *db;

    vector<vector<NNGGrid>> grids;                                                             // for buildTreeUsingNearestNeighborGraph only
    vector<TreeNode *> globalTreeNodes;                                                        // for buildTreeUsingNearestNeighborGraph only, make sure that TreeNode->id == the index of the TreeNode in globalTreeNodes!!
    vector<bool> globalMerged;                                                                 // for buildTreeUsingNearestNeighborGraph only
    priority_queue<NngNodePair, vector<NngNodePair>, greater<NngNodePair>> globalMinimumPairs; // for buildTreeUsingNearestNeighborGraph only
    int gridCountX;                                                                            // how many grids in X direction,  for buildTreeUsingNearestNeighborGraph only
    int gridCountY;                                                                            // for buildTreeUsingNearestNeighborGraph only

    void init()
    {
        root = NULL;
        leafCount = 0;
        nodeCount = 0;
        idToTreeNodeMap.clear();
        grids.clear();
        globalTreeNodes.clear();
        db = NULL;
    }
    void buildTopoUsingNearestNeighborGraph();
    double nearestNeighborPairCost(NngNode, NngNode);
    double nearestNeighborPairCost(TreeNode *, TreeNode *);

    void initTreeNodes(); // for buildTreeUsingNearestNeighborGraph only
    void buildTreeUsingNearestNeighborGraph();
    void calculateNearestNeighbor();

    void initGrids();                     // for calculateNearestNeighbor_Grid() only
    void calculateNearestNeighbor_Grid(); // see the paper: clock aware low power placement algorithm 2, for buildTreeUsingNearestNeighborGraph only
    void updateVforOneGrid(NNGGrid &, vector<TreeNode *>);
    void readTopologyFromFile();
};

class NNGGrid
{
public:
    NNGGrid()
    {
        Init();
    }
    void clearV()
    {
        priority_queue<NngNodePair, vector<NngNodePair>, greater<NngNodePair>> empty;
        swap(empty, V);
    }
    void Init()
    {
        gridTreeNodes.clear();
        clearV();
        center.SetZero();
        ll.SetZero();
        ur.SetZero();
        area = 0;
        width = 0;
        height = 0;
    }
    Point_2D center;
    Point_2D ll;
    Point_2D ur;
    float width;
    float height;
    float area;
    vector<TreeNode *> gridTreeNodes;
    priority_queue<NngNodePair, vector<NngNodePair>, greater<NngNodePair>> V; // follow the denotion in the paper: clock aware low power placement
    double getArea()
    {
        area = width * height;
        return width * height;
    }
};
#endif