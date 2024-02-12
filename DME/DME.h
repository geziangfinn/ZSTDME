#ifndef DME_H
#define DME_H
#include "global.h"
#include "arghandler.h"
#include "objects.h"
#include "ctsdb.h"
#include "topology.h"
#include "plotter.h"

#define LINEAR_DELAY 0
#define ELMORE_DELAY 1
#define UNIT_CAPACITANCE 1
#define UNIT_RESISTANCE 1

class ZSTDMERouter
{
public:
    ZSTDMERouter()
    {
        init();
    }
    ZSTDMERouter(CTSDB *_db)
    {
        init();
        db = _db;
    }

    int sinkCount;  // test 1
    int delayModel; // 0 for linear and 1 for elmore

    TreeTopology *topology;

    double totalWirelength;

    vector<Point_2D> treeNodeLocation; // also known as pl in the ZST DME paper by abk
    vector<SteinerPoint *> solution;

    CTSDB *db;

    void init()
    {
        db = NULL;
        sinkCount = 0;
        delayModel = -1;

        treeNodeLocation.clear();
        solution.clear();

        topology = NULL;
        totalWirelength = 0;
    }

    void setTopology(TreeTopology *_topology)
    {
        topology = _topology;
    }
    void setDelayModel(int _delayModel)
    {
        delayModel = _delayModel;
    }
    // Generate embedding
    void ZSTDME(); // Deferred-Merge Embedding
    void topDown();
    void bottomUp();

    void drawBottomUp();
    void drawBottomUpMerge(string name, TRR trr1, TRR trr2, Segment merge);
    void drawTRRPair(string name, TRR trr1, TRR trr2);

    void buildSolution();
    void drawSolution(); 

    Segment TRRintersectTRR(TRR &trr1, TRR &trr2);

    void updateMergeCapacitance(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
    void updateMergeDelay(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
    double solveForX(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L);
    double solveForLPrime(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag); // see κ prime in the abk paper
};

class BSTDMERouter
{
};
#endif