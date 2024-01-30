#ifndef DME_H
#define DME_H
#include "global.h"
#include "arghandler.h"
#include "objects.h"
#include "ctsdb.h"

#define LINEAR_DELAY 0
#define ELMORE_DELAY 1

class ZSTDMERouter
{
public:
    ZSTDMERouter()
    {
        init();
    }
    ZSTDMERouter(CTSDB *_db)
    {
        db = _db;
        init();
    }

    int sinkCount;  // test 1
    int delayModel; // 0 for linear and 1 for elmore

    vector<Segment> vertexMS; // set of segments
    vector<TRR> vertexTRR;
    vector<double> vertexDistE;
    TreeTopology *topology;

    double totalWirelength;

    // string _RUNDIR = "../run_tmp/";
    // string _DRAWDIR = "../draw/";

    vector<Point_2D> treeNodeLocation;
    vector<SteinerPoint *> solution;

    CTSDB *db;

    void init()
    {
        db = NULL;
        sinkCount = 0;
        delayModel = 1;

        vertexMS.clear();
        vertexTRR.clear();
        vertexDistE.clear();
        treeNodeLocation.clear();
        solution.clear();

        topology = NULL;
        totalWirelength = 0;
    }

    // Generate embedding
    void ZSTDME(); // Deferred-Merge Embedding

    void topDown();
    void bottomUp();

    void route();
    void buildSolution();
    void buildSolution_ISPD();
    void reportTotalWL();
    void writeSolution();
    void buildTopology_nngraph(); // nn for nearest neighbor
    void buildTopology_from_binary();
    void DLE_3D();
    void DLE_loop(TreeNode *node);
    void NearestAssign(TreeNode *node);
    void setdelay_model(int);
    void draw_bottom_up();
    void draw_solution();
    void draw_TRR_pair(TRR trr1, TRR trr2);
    void draw_blockages();
    void bouncing_check();
    void count_TSV(); //! this function is for 2-layer chip only

    Segment TRRintersect(TRR &trr1, TRR &trr2);

    double calc_x_RC(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L);
    double calc_L2_RC(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag);
    void update_merge_Capacitance(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
    void update_merge_Delay(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
    double calc_standard_Capacitance(double capacitance_in_fF);
    float calc_delay_RLC(TreeNode *nodeMerge, TreeNode *nodeChild, float WL);
    void RLC_calculation(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double &ea, double &eb);
    void modify_coord_by_L1(double &ea, double &eb, TreeNode *nodeLeft, TreeNode *nodeRight, float x);
    void modify_coord_by_L2(double &ea, double &eb, TreeNode *nodeLeft, TreeNode *nodeRight, float x);
    void initiate_parameters();
    void metalLayerCal();
};

class BSTDMERouter
{
};
#endif