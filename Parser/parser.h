#ifndef PARSER_H
#define PARSER_H
#include "global.h"
#include "objects.h"
#include "ctsdb.h"
class ISPD2009Parser
{
public:
    string filePath;
    ISPD2009Parser()
    {
    }
    void ReadFile(string filePath, CTSDB * db);
};
#endif