#ifndef DME_H
#define DME_H
#include "global.h"
#include "arghandler.h"
#include "objects.h"
#include "ctsdb.h"
#include "topology.h"
#include "plotter.h"

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
    void repairSolution();// eliminate overlap between clock tree and obstacles, performed after buildSolution. Ignore its impact on skew for now. It will cause higher wirelength, for sure

    void drawBottomUp();
    void drawBottomUpMerge(string name, TRR trr1, TRR trr2, Segment merge);

    void buildSolution();
    void drawSolution();

    Segment nineRegionBasedFeasibleMergeSegmentCutting(Segment, Segment); // See the paper in README
};

class BSTDMERouter
{
};
#endif