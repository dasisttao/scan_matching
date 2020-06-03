#include <iostream>
#include <fstream>
#include "csvfile.h"

using namespace std;

namespace OutputCSV
{
    void outputCSV()
    {
        try
        {
            // csvfile csv("MyTable.csv"); // throws exceptions!
            // csv << "X" << "VALUE" << endrow;
        }
        catch (const std::exception &ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }
} // namespace OutputCSV