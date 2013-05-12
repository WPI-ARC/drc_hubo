#include <stdlib.h>
#include <stdio.h>
#include <vector>

#ifndef DTW_H
#define DTW_H

namespace DTW
{

class SimpleDTW
{
private:

    double (*DistanceFn)(std::vector<double> p1, std::vector<double> p2);
    double ** dtw_matrix;
    unsigned int x_dim;
    unsigned int y_dim;
    bool initialized;

public:

    SimpleDTW(unsigned int x_size, unsigned int y_size, double (*distance_eval)(std::vector<double> p1, std::vector<double> p2));

    SimpleDTW()
    {
    }

    SimpleDTW(double (*distance_eval)(std::vector<double> p1, std::vector<double> p2));

    ~SimpleDTW();

    bool Initialize(unsigned int x_size, unsigned int y_size);

    double EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2);

};

}

#endif // DTW_H
