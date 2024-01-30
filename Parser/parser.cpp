#include "parser.h"

void ISPD2009Parser::ReadFile(string filePath, CTSDB *db)
{
    ifstream file(filePath);
    string line, label;

    while (getline(file, line))
    {
        if (line == "" || line[0] == '#')
            continue;
        stringstream ss(line);
        ss >> label;
        if (label == "num")
        { // Netlist (Insts)
            ss >> label;
            if (label == "sink")
            {
                int numSink;
                ss >> numSink;
                int countInst = 0;
                while (countInst < numSink)
                {
                    getline(file, line);
                    if (line == "")
                        continue;
                    stringstream ss2(line);
                    // add inst
                    Sink tempsink;
                    ss2 >> tempsink.name>> tempsink.x >> tempsink.y>>tempsink.capacitance;//2D
                    tempsink.id=countInst;
                    db->dbSinks.emplace_back(tempsink);
                    countInst++;
                }
            }
            else if (label == "blockage")
            {
                int numBlockage;
                ss >> numBlockage;
                int countInst = 0;
                while (countInst < numBlockage)
                {
                    getline(file, line);
                    if (line == "")
                        continue;
                    stringstream ss2(line);
                    // add inst
                    Blockage tempblockage;
                    // ss2 >> tempblockage.ll.x >> tempblockage.ll.y >> tempblockage.ur.x >> tempblockage.ur.y >> tempblockage.layer; // 3D
                    ss2 >> tempblockage.ll.x >> tempblockage.ll.y >> tempblockage.ur.x >> tempblockage.ur.y;//2D
                    db->dbBlockages.emplace_back(tempblockage);
                    countInst++;
                }
            }
        }
    }
}