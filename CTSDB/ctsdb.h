#ifndef CTSDB_H
#define CTSDB_H
#include "global.h"
#include "objects.h"
class CTSDB
{
public:
    CTSDB()
    {
        init();
    }
    vector<Sink> dbSinks;
    vector<Blockage> dbBlockages;
    Point_2D clockSource;
    vector<metal> dbMetals;
    metal TSV;

    void init()
    {
        clockSource.SetZero();
        dbSinks.clear();
        dbBlockages.clear();
    }

    void showCTSdbInfo();
};
#endif