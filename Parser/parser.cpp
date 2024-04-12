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
                    ss2 >> tempsink.name >> tempsink.x >> tempsink.y >> tempsink.capacitance; // 2D
                    tempsink.id = countInst;
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
                    ss2 >> tempblockage.ll.x >> tempblockage.ll.y >> tempblockage.ur.x >> tempblockage.ur.y; // 2D
                    db->dbBlockages.emplace_back(tempblockage);
                    countInst++;
                }
            }
        }
    }
}

void ISPD2009Parser_3D::ReadFile(string filePath, CTSDB *db)
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
                int countInst = 0;// sink id start from 0 here! but should start from 1 in the output file
                while (countInst < numSink)
                {
                    getline(file, line);
                    if (line == "")
                        continue;
                    stringstream ss2(line);
                    // add inst
                    Sink tempsink;
                    string dummy;
                    ss2 >> tempsink.name >> dummy >> tempsink.x >> tempsink.y >> tempsink.layer >> tempsink.capacitance; // 3D
                    // tempsink.capacitance *= 1e-15;
                    tempsink.id = countInst;
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
                    ss2 >> tempblockage.ll.x >> tempblockage.ll.y >> tempblockage.ur.x >> tempblockage.ur.y; // 2D
                    db->dbBlockages.emplace_back(tempblockage);
                    countInst++;
                }
            }
            else if (label == "wirelib")
            {
                int metalCount;
                ss >> metalCount;
                int countInst = 0;
                while (countInst < metalCount - 1)
                {
                    getline(file, line);
                    if (line == "")
                        continue;
                    stringstream ss2(line);
                    // add inst
                    metal tempMetal;
                    string dummy;
                    // ss2 >> tempblockage.ll.x >> tempblockage.ll.y >> tempblockage.ur.x >> tempblockage.ur.y >> tempblockage.layer; // 3D
                    ss2 >> dummy >> tempMetal.rw >> tempMetal.cw >> tempMetal.lw; // 2D
                    // tempMetal.cw *= 1e-15;
                    tempMetal.lw *= 1e15;
                    tempMetal.rw *= 1e15;
                    db->dbMetals.emplace_back(tempMetal);
                    countInst++;
                }
                getline(file, line);
                stringstream ss2(line);
                metal TSV;
                string dummy;
                ss2 >> dummy >> TSV.rw >> TSV.cw >> TSV.lw;
                // TSV.cw *= 1e-15;
                TSV.lw*=1e15;
                TSV.rw*=1e15;
                db->TSV = TSV;
            }
        }
    }
    for (metal curMetal : db->dbMetals)
    {
        cout << curMetal.rw << " " << curMetal.cw << " " << curMetal.lw << endl;
    }
    cout << "TSV: " << db->TSV.rw << " " << db->TSV.cw << " " << db->TSV.lw << endl;
}
