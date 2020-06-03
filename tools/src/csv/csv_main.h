#include <iostream>
#include <fstream>
#include "csvfile.h"
#include <ukf/ukf.h>
#include <algorithm>
#include <string>
#include <iterator>
#include <fstream>
#include <vector>

using namespace std;
using namespace std;
using namespace boost;

namespace WriteCSV
{
    void kalmanCSV(const UKF &ukf_filter)
    {
        try
        {
            csvfile csv("KalmanFilterState.csv"); // throws exceptions!
            csv << ukf_filter.x_(0) << ukf_filter.x_(1) << ukf_filter.x_(2) << ukf_filter.x_(3) << ukf_filter.x_(4) << endrow;
            csv << ukf_filter.P_(0, 0) << ukf_filter.P_(1, 0) << ukf_filter.P_(2, 0) << ukf_filter.P_(3, 0) << ukf_filter.P_(4, 0) << endrow;
            csv << ukf_filter.P_(0, 1) << ukf_filter.P_(1, 1) << ukf_filter.P_(2, 1) << ukf_filter.P_(3, 1) << ukf_filter.P_(4, 1) << endrow;
            csv << ukf_filter.P_(0, 2) << ukf_filter.P_(1, 2) << ukf_filter.P_(2, 2) << ukf_filter.P_(3, 2) << ukf_filter.P_(4, 2) << endrow;
            csv << ukf_filter.P_(0, 3) << ukf_filter.P_(1, 3) << ukf_filter.P_(2, 3) << ukf_filter.P_(3, 3) << ukf_filter.P_(4, 3) << endrow;
            csv << ukf_filter.P_(0, 4) << ukf_filter.P_(1, 4) << ukf_filter.P_(2, 4) << ukf_filter.P_(3, 4) << ukf_filter.P_(4, 4) << endrow;
        }
        catch (const std::exception &ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }
} // namespace WriteCSV

namespace ReadCSV
{

    void kalmanCSV(UKF &ukf_filter)
    {
        std::ifstream data("KalmanFilterState.csv");
        std::string line;
        std::vector<std::vector<std::string>> parsedCsv;
        while (std::getline(data, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            std::vector<std::string> parsedRow;
            while (std::getline(lineStream, cell, ';'))
            {
                parsedRow.push_back(cell);
            }
            parsedCsv.push_back(parsedRow);
        }
        int col = 0;

        for (vector<string> strings : parsedCsv)
        {
            cout << strings.size() << endl;
            if (col == 0)
            {
                ukf_filter.x_(0) = stod(strings[0]);
                ukf_filter.x_(1) = stod(strings[1]);
                ukf_filter.x_(2) = stod(strings[2]);
                ukf_filter.x_(3) = stod(strings[3]);
                ukf_filter.x_(4) = stod(strings[4]);
            }
            if (col == 1)
            {
                ukf_filter.P_(0, 0) = stod(strings[0]);
                ukf_filter.P_(1, 0) = stod(strings[1]);
                ukf_filter.P_(2, 0) = stod(strings[2]);
                ukf_filter.P_(3, 0) = stod(strings[3]);
                ukf_filter.P_(4, 0) = stod(strings[4]);
            }
            if (col == 2)
            {
                ukf_filter.P_(0, 1) = stod(strings[0]);
                ukf_filter.P_(1, 1) = stod(strings[1]);
                ukf_filter.P_(2, 1) = stod(strings[2]);
                ukf_filter.P_(3, 1) = stod(strings[3]);
                ukf_filter.P_(4, 1) = stod(strings[4]);
            }
            if (col == 3)
            {
                ukf_filter.P_(0, 2) = stod(strings[0]);
                ukf_filter.P_(1, 2) = stod(strings[1]);
                ukf_filter.P_(2, 2) = stod(strings[2]);
                ukf_filter.P_(3, 2) = stod(strings[3]);
                ukf_filter.P_(4, 2) = stod(strings[4]);
            }
            if (col == 4)
            {
                ukf_filter.P_(0, 3) = stod(strings[0]);
                ukf_filter.P_(1, 3) = stod(strings[1]);
                ukf_filter.P_(2, 3) = stod(strings[2]);
                ukf_filter.P_(3, 3) = stod(strings[3]);
                ukf_filter.P_(4, 3) = stod(strings[4]);
            }
            if (col == 5)
            {
                ukf_filter.P_(0, 4) = stod(strings[0]);
                ukf_filter.P_(1, 4) = stod(strings[1]);
                ukf_filter.P_(2, 4) = stod(strings[2]);
                ukf_filter.P_(3, 4) = stod(strings[3]);
                ukf_filter.P_(4, 4) = stod(strings[4]);
            }
            col++;
        }
        cout << ukf_filter.x_ << endl;
        cout << ukf_filter.P_ << endl;
    }

} // namespace ReadCSV
