#pragma once
#include <iostream>
#include "matplotlibcpp.h"
using namespace std;
namespace plt = matplotlibcpp;
class Plotter
{

public:
    void plottest()
    {
        plt::plot({1, 3, 2, 4});
        plt::show();
    }
};