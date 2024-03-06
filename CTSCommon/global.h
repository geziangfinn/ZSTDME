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
#include <queue>
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

#define EPS 1e-5 // 1e-2, for now
const string padding(30, '=');

#define LINEAR_DELAY 0
#define ELMORE_DELAY 1
#define UNIT_CAPACITANCE 1
#define UNIT_RESISTANCE 1

enum REGION
{
    DUMMY,
    UP_LEFT,
    UP_MID,
    UP_RIGHT,
    MID_LEFT,
    MID_MID,
    MID_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_MID,
    BOTTOM_RIGHT
    // 1 2 3
    // 4 5 6
    // 7 8 9
    //  here we start from 1, to be consistent with the sung kyu lym paper
};

enum POSTION_VERTICAL
{
    UP,
    MID_VERTICAL,
    BOTTOM
};

enum POSTION_HORIZONTAL
{
    LEFT,
    MID_HORIZONTAL,
    RIGHT
};

enum DIRECTION
{
    VERTICAL,
    HORIZONTAL
};

inline bool double_equal(double a, double b)
{
    return fabs(a - b) <= EPS;
}

class Point_2D
{
public:
    double x, y;
    Point_2D() { SetZero(); }
    Point_2D(double _x, double _y) : x(_x), y(_y)
    {
    }
    void SetZero()
    {
        x = y = 0.0; //!! 0.0!!!!
    }
    bool operator==(const Point_2D &rhs) const { return double_equal(x, rhs.x) && double_equal(y, rhs.y); }
    friend inline std::ostream &operator<<(std::ostream &os, const Point_2D &gp)
    {
        os << "(" << gp.x << "," << gp.y << ")";
        return os;
    }
};

class Rect
{
public:
    Rect()
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
    double getWidth()
    {
        double width = ur.x - ll.x;
        assert(width > 0.0);
        return width;
    }
    double getHeight()
    {
        double height = ur.y - ll.y;
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
    double getArea()
    {
        return getHeight() * getWidth();
    }
};

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

class Interval
{
public:
    Interval()
    {
        Init();
    }
    Interval(double _start, double _end)
    {
        if (double_greater(_start, _end))
        {
            std::swap(_start, _end);
        }
        start = _start;
        end = _end;
    }
    void Init()
    {
        start = 0;
        end = 0;
    }

    bool inside(double xORy)
    {
        return (double_less(start, xORy) && double_less(xORy, end)); //! less, not less or equal!(open interval)
    }

    bool include(Interval interval)
    {
        return (double_lessorequal(this->start, interval.start) && double_lessorequal(interval.end, this->end));
    }

    double start;
    double end;
};

class RectilinearLine
{
public:
    RectilinearLine()
    {
        Init();
    }
    void Init()
    {
        ll.SetZero();
        ur.SetZero();
        direction = -1;
    };

    RectilinearLine(Point_2D point1, Point_2D point2)
    {
        if (double_equal(point1.x, point2.x))
        {
            direction = VERTICAL;
            ll.x = ur.x = point1.x;
            ll.y = min(point1.y, point2.y);
            ur.y = max(point1.y, point2.y);
        }
        else if (double_equal(point1.y, point2.y))
        {
            direction = HORIZONTAL;
            ll.y = ur.y = point1.y;
            ll.x = min(point1.x, point2.x);
            ur.x = max(point1.x, point2.x);
        }
        else
        {
            cerr << "NOT A RECTILINEAR LINE!\n";
            exit(0);
        }
    }

    Point_2D ll; // lower OR left, not lower left because this is a line
    Point_2D ur; // upper OR right
    int direction;
};
#endif
