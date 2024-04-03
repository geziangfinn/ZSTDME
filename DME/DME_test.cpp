#include "global.h"
#include "parser.h"
#include "objects.h"
#include "topology.h"
#include "DME.h"
using namespace std;
int main(int argc, char *argv[])
{
    ISPD2009Parser parser;
    CTSDB *ctsdb = new CTSDB();
    gArg.Init(argc, argv);
    if (argc < 2)
    {
        return 0;
    }
    if (strcmp(argv[1] + 1, "input") == 0) // -input, argv[1]=='-'
    {
        // bookshelf
        printf("Use ISPD2009_3D format\n");

        string filename = argv[2];
        string::size_type pos = filename.rfind("/");
        if (pos != string::npos)
        {
            printf("    Path = %s\n", filename.substr(0, pos + 1).c_str());
            gArg.Override("path", filename.substr(0, pos + 1));
        }
        string benchmarkName;
        int length = filename.length();
        benchmarkName = filename.substr(pos + 1, length - pos);
        gArg.Override("benchmarkName", benchmarkName);
        cout << "    Benchmark: " << benchmarkName << endl;

        string plotPath;

        if (!gArg.GetString("plotPath", &plotPath))
        {
            plotPath = "./";
        }

        plotPath += "/" + benchmarkName + "/";

        string cmd = "rm -rf " + plotPath;
        system(cmd.c_str());
        cmd = "mkdir -p " + plotPath;
        system(cmd.c_str());

        gArg.Override("plotPath", plotPath);

        cout << "    Plot path: " << plotPath << endl;

        parser.ReadFile(argv[2], ctsdb);
    }
    ctsdb->showCTSdbInfo();
    TreeTopology *topo = new TreeTopology(ctsdb);
    // topo->buildTreeUsingNearestNeighborGraph();
    topo->buildTreeUsingNearestNeighborGraph_BucketDecomposition();
    ZSTDMERouter *router = new ZSTDMERouter(ctsdb);
    int delayModel=ELMORE_DELAY;
    if(gArg.CheckExist("linearDelay"))
    {
        delayModel=LINEAR_DELAY;
    }
    router->setDelayModel(delayModel);
    router->setTopology(topo);

    //router->ZSTDME();
    router->drawBottomUp();
    cout<<"drawn bottomup"<<endl;
    // 2. Find Exact Node Location(top down)
    router->topDown();

    cout << padding << "Finished DME" << padding << endl;
    router->buildSolution();
    router->drawSolution();
}