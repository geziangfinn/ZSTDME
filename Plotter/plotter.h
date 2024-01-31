#ifndef PLOTTER_H
#define PLOTTER_H
#include "global.h"
#include "arghandler.h"
using namespace std;
inline void plotBoxPLT(ofstream& stream, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    stream << x1 << ", " << y1 << endl << x2 << ", " << y2 << endl
           << x3 << ", " << y3 << endl << x4 << ", " << y4 << endl
           << x1 << ", " << y1 << endl << endl;
}

inline void plotLinePLT(ofstream& stream, double x1, double y1, double x2, double y2)
{
    stream << x1 << ", " << y1 << endl << x2 << ", " << y2 << endl<<endl;
}
#endif