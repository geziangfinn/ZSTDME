#include "global.h"
#include "parser.h"
#include "objects.h"
using namespace std;
int main(int argc, char *argv[])
{
    ISPD2009Parser_3D parser;
    CTSDB* ctsdb=new CTSDB();
    gArg.Init(argc, argv);
    if (argc < 2)
    {
        return 0;
    }
    if (strcmp(argv[1] + 1, "input") == 0) // -aux, argv[1]=='-'
    {
        // bookshelf
        printf("Use ISPD2009_3D format\n");

        string filename = argv[2];
        string::size_type pos = filename.rfind("/");
        string benchmarkName;
        int length = filename.length();
        gArg.Override("benchmarkName", benchmarkName);
        benchmarkName = filename.substr(pos + 1, length - pos);
        
        if (pos != string::npos)
        {
            printf("    Path = %s\n", filename.substr(0, pos + 1).c_str());
            gArg.Override("path", filename.substr(0, pos + 1));
        }

        parser.ReadFile(argv[2], ctsdb);
    }
    ctsdb->showCTSdbInfo();
}