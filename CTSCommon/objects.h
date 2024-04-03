#ifndef OBJECTS_H
#define OBJECTS_H
#include "global.h"
#include "arghandler.h"

struct metal
{
    double cw; // unit capacitance of metal
    double rw; // unit resistance of metal
    double lw; // unit inductance of metal
};

struct wire
{
    int left_id;
    int right_id;
    int metal_index;
};

class Sink : public Point_2D
{
public:
    string name;
    int id;
    double capacitance;
    int layer; // layer in 3D IC
    Sink() {}
    Sink(int _id, double _x, double _y) : id(_id)
    {
        init();
        x = _x;
        y = _y;
    }
    Sink(int _id, double _x, double _y, double _capacitance) : id(_id)
    {
        init();
        x = _x;
        y = _y;
        capacitance = _capacitance;
    }
    void init()
    {
        name = "";
        id = -1;
        capacitance = 0.0;
        layer = -1;
    }

    string str_xy()
    {
        stringstream s;
        s << "[" << x << "," << y << "]";
        return s.str();
    }
    friend inline std::ostream &operator<<(std::ostream &os, const Sink &sink)
    {
        os << "Sink " << sink.id << ", x: " << sink.x << ", y: " << sink.y;
        return os;
    }
};

class SteinerPoint : public Point_2D
{
public:
    SteinerPoint *leftChild;
    SteinerPoint *rightChild;
    SteinerPoint *parent;

    SteinerPoint(Point_2D p)
    {
        x = p.x;
        y = p.y;
        leftChild = NULL;
        rightChild = NULL;
        parent = NULL;
    }

    void setLeftChild(SteinerPoint *child) { leftChild = child; }
    void setRightChild(SteinerPoint *child) { rightChild = child; }
    void setParent(SteinerPoint *p) { parent = p; }
};

class Segment
{
public:
    Point_2D lowerPoint, higherPoint; // lowerPoint has lower y
    int id;                           // unique id for each segment, -1 for not valid, 0 for valid, -2 for point intersection
    Segment()
    {
        init();
    }
    Segment(Point_2D u, Point_2D v) : lowerPoint(u), higherPoint(v)
    {

        if (lowerPoint.y > higherPoint.y) // assume lowerPoint is the lower point
        {
            std::swap(lowerPoint, higherPoint);
        }
    }

    Segment(Sink leaf) : lowerPoint(leaf), higherPoint(leaf)
    {
        if (lowerPoint.y > higherPoint.y)
        {
            std::swap(lowerPoint, higherPoint);
        }
    }
    void init()
    {
        lowerPoint.SetZero();
        higherPoint.SetZero();
        id = -1;
    }

    bool isPoint() { return lowerPoint == higherPoint; }
    bool operator==(const Segment &rhs) const { return (lowerPoint == rhs.lowerPoint && higherPoint == rhs.higherPoint); }

    friend inline std::ostream &operator<<(std::ostream &os, const Segment &seg)
    {
        os << "Seg: (" << seg.lowerPoint << "," << seg.higherPoint << ")";
        return os;
    }
    double slope(); // get slope of segment
};

class TRR
{
public:
    Segment core;
    double radius;
    TRR()
    {
        init();
    }
    TRR(Segment _segment, double _radius) : core(_segment), radius(_radius) {}
    void init()
    {
        radius = 0;
        core = Segment();
    }
    friend inline std::ostream &operator<<(std::ostream &os, const TRR &trr)
    {
        os << trr.core << "; radius:" << trr.radius;
        return os;
    }

    void drawTRR(ofstream &stream);
    void drawCore(ofstream &stream);

    bool insideTRR(Point_2D point);
    Point_2D getMiddlePoint();
    vector<Segment> getBoundarys();
};

class TreeNode
{
public:
    int id;
    double loadCapacitance; // load from taps
    double delay;
    bool buffered = false;
    TRR trr;
    TreeNode *leftChild;
    TreeNode *rightChild;
    TreeNode *parent;
    int level;           // level in the binary tree
    int layer;           // layer in 3D IC
    int metalLayerIndex; // indicate the metal used  to connect this node with its childs
    pair<int, int> el;   // for DLE, see the DLE paper
    TreeNode(int _id)
    {
        init();
        id = _id;
    }
    void init()
    {
        leftChild = NULL;
        rightChild = NULL;
        parent = NULL;
        trr = TRR();
        delay = 0.0;
        level = -1;
        layer = -1;
        metalLayerIndex = -1;
        el = make_pair(-1, -1);
        buffered = false;
    }

    void set_lc(TreeNode *child) { leftChild = child; }
    void set_rc(TreeNode *child) { rightChild = child; }
    void set_par(TreeNode *p) { parent = p; }
    bool isLeaf() { return leftChild == NULL && rightChild == NULL; }
};

class Blockage : public Rect
{
};

inline double minManhattanDist(Segment ms1, Segment ms2)
{
    // ms: merge segment in ZST/DME
    double distance = min(L1Dist(ms1.lowerPoint, ms2.lowerPoint), L1Dist(ms1.lowerPoint, ms2.higherPoint));
    distance = min(distance, L1Dist(ms1.higherPoint, ms2.lowerPoint));
    distance = min(distance, L1Dist(ms1.higherPoint, ms2.higherPoint)); // but why need to calc 2*2 possiblity?
    assert(distance > 0);
    return distance;
}

inline double minManhattanDist(TreeNode *nodeLeft, TreeNode *nodeRight)
{
    // ms: merge segment in ZST/DME
    auto ms1 = nodeLeft->trr.core;
    auto ms2 = nodeRight->trr.core;
    // get |e_a|, |e_b|
    // double distance = min(L1Dist(ms1.lowerPoint, ms2.lowerPoint), L1Dist(ms1.lowerPoint, ms2.higherPoint));
    // distance = min(distance, L1Dist(ms1.higherPoint, ms2.lowerPoint));
    // distance = min(distance, L1Dist(ms1.higherPoint, ms2.higherPoint)); // but why need to calc 2*2 possiblity?
    // assert(distance > 0);
    double distance = minManhattanDist(ms1, ms2);
    return distance;
}

vector<Segment> segmentCutByBlockage(Segment, Blockage);

vector<Segment> segmentCutBy2ParallelLines(Segment, double, double, bool); // true for vertical lines, false for horizontal lines,
                                                                           // this function is to avoid removing duplicate segments
                                                                           // when cutting a segment with 2 parallel lines,
                                                                           // which is a time consuming operation

vector<Segment> segmentCutByLine(Segment, double, bool); // true for vertical lines, false for horizontal lines

int regionNumber(Segment, Blockage);

bool insideRect(Point_2D, Rect);

Point_2D moveToClosestBoundary(Point_2D, Rect);

void updateMergeCapacitance(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
void updateMergeDelay(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb);
double solveForX(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L);
double solveForLPrime(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag); // see κ prime in the abk paper
Segment TRRintersectTRR(TRR trr1, TRR trr2);
void drawTRRPair(string name, TRR trr1, TRR trr2);
void TRRBasedMerge(TreeNode *, TreeNode *, TreeNode *);
bool segmentOnSameLine(Segment, Segment);
Segment TRRintersectSeg(TRR trr, Segment seg);
Segment intersect(Segment lhs, Segment rhs); // lhs: left hand side, rhs: right hand side

double solveForX_multiMetal(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, double L, vector<metal> metals, metal TSV);
double solveForLPrime_multiMetal(TreeNode *nodeLeft, TreeNode *nodeRight, TreeNode *nodeMerge, int tag, vector<metal> metals, metal TSV); // see κ prime in the abk paper
void updateMergeCapacitance_multiMetal(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb, vector<metal> metals, metal TSV);
void updateMergeDelay_multiMetal(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double ea, double eb, vector<metal> metals, metal TSV);

void RLCCalculation(TreeNode *nodeMerge, TreeNode *nodeLeft, TreeNode *nodeRight, double &ea, double &eb);
#endif