#ifndef GLOBAL_H
#define GLOBAL_H

// STL libraries
#include <iostream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <sstream>
#include <memory.h>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <cmath>
#include <functional>
#include <limits>
#include "arghandler.h"

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

using namespace std;

#define EPS 1.0E-6 // 1e-6, for now
const string padding(30, '=');

class Point_2D
{
public:
    double x, y;
    Point_2D() { init(); }
    Point_2D(double _x, double _y) : x(_x), y(_y)
    {
    }
    void init()
    {
        x = 0;
        y = 0;
    }
    void SetZero()
    {
        x = y = 0.0; //!! 0.0!!!!
    }
    bool operator==(const Point_2D &rhs) const { return x == rhs.x && y == rhs.y; }
    friend inline std::ostream &operator<<(std::ostream &os, const Point_2D &gp)
    {
        os << "(" << gp.x << "," << gp.y << ")";
        return os;
    }
};

class CRect
{
public:
    CRect()
    {
        Init();
    }
    void Print()
    {
        cout << "lower left: " << ll << " to upper right: " << ur << "\n";
    }
    void Init()
    {
        ll.SetZero();
        ur.SetZero();
    }
    Point_2D ll; // ll: lower left coor
    Point_2D ur; // ur: upper right coor
    float getWidth()
    {
        float width = ur.x - ll.x;
        assert(width > 0.0);
        return width;
    }
    float getHeight()
    {
        float height = ur.y - ll.y;
        assert(height > 0.0);
        return height;
    }
    Point_2D getCenter()
    {
        Point_2D center = ll;
        center.x += 0.5 * this->getWidth();
        center.y += 0.5 * this->getHeight();
        return center;
    }
    float getArea()
    {
        return getHeight() * getWidth();
    }
};

inline bool double_equal(double a, double b)
{
    return fabs(a - b) < EPS;
}

inline bool double_greater(double a, double b) // return true if a > b
{
    return a - b > 1.0 * EPS;
}

inline bool double_less(double a, double b) // return true if a < b
{
    return a - b < -1.0 * EPS;
}

inline bool double_lessorequal(double a, double b)
{
    return double_less(a, b) || double_equal(a, b);
}

inline bool double_greaterorequal(double a, double b)
{
    return double_greater(a, b) || double_equal(a, b);
}

inline double L1Dist(Point_2D p1, Point_2D p2) { return abs(p1.x - p2.x) + abs(p1.y - p2.y); }

#endif
