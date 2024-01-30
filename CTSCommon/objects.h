#ifndef OBJECTS_H
#define OBJECTS_H
#include "global.h"
#include "arghandler.h"
class Sink : public Point_2D
{
public:
    string name;
    int id;
    double capacitance;
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
        name="";
        id=-1;
        capacitance=0.0;
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
    int id;          // unique id for each segment
    Segment()
    {
        init();
    }
    Segment(Point_2D u, Point_2D v) : lowerPoint(u), higherPoint(v)
    {
        if (lowerPoint.y > higherPoint.y) // assume lowerPoint is the lower point
        {
            swap(lowerPoint, higherPoint);
        }
    }

    Segment(Sink leaf) : lowerPoint(leaf), higherPoint(leaf)
    {
        if (lowerPoint.y > higherPoint.y)
        {
            swap(lowerPoint, higherPoint);
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
    double slope();                 // get slope of segment
    Segment intersect(Segment rhs); // rhs: right hand side
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

    Segment intersectTRR(Segment &seg); //! this function is used for TRRs in top-down phase,
                                        //! in which all cores of TRRs are points.
                                        //! And the intersection point is used as solution,
                                        //! which means ignore the other points on ms(v)
};

class TreeNode
{
public:
    int id;
    double load_capacitance; // load from taps
    double delay;
    bool buffered = false;
    TRR trr;
    TreeNode *leftChild;
    TreeNode *rightChild;
    TreeNode *parent;
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
    }

    void set_lc(TreeNode *child) { leftChild = child; }
    void set_rc(TreeNode *child) { rightChild = child; }
    void set_par(TreeNode *p) { parent = p; }
    bool isLeaf() { return leftChild == NULL && rightChild == NULL; }
};

class TreeTopology
{
public:
    TreeNode *root;
    int leafCount;
    int nodeCount;
    unordered_map<int, TreeNode *> idToTreeNodeMap;

    TreeTopology() {}
    void init()
    {
        root = NULL;
        leafCount = 0;
        nodeCount = 0;
        idToTreeNodeMap.clear();
    }

    void inittree_from_binary(vector<Sink> taps, int sz, vector<int> preOrder, vector<int> inOrder);
    void inittree_nng(vector<Sink> taps);
    void constructTree_old(bool modifyCurrentTree = false);
    void constructTree_from_binary(vector<Sink> taps, vector<int> preOrder, vector<int> inOrder);
    void layerassignment(vector<pair<int, int>> IdAndLayer);
    void treeLayerCal();
    TreeNode *buildTree_from_binary(vector<Sink> taps, vector<int> pre, vector<int> in, int preStart, int preEnd, int inStart, int inEnd);
};

class Blockage : public CRect
{
};

inline double minManhattanDist(TreeNode *nodeLeft, TreeNode *nodeRight)
{
    // ms: merge segment in ZST/DME
    auto ms1 = nodeLeft->trr.core;
    auto ms2 = nodeRight->trr.core;
    // get |e_a|, |e_b|
    double distance = min(L1Dist(ms1.lowerPoint, ms2.lowerPoint), L1Dist(ms1.lowerPoint, ms2.higherPoint));
    distance = min(distance, L1Dist(ms1.higherPoint, ms2.lowerPoint));
    distance = min(distance, L1Dist(ms1.higherPoint, ms2.higherPoint)); // but why need to calc 2*2 possiblity?
    assert(distance > 0);
    return distance;
}

#endif