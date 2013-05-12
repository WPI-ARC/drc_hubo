#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include "math.h"
#include <vector>
#include <string>
#include <sstream>
#include "string.h"
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include "trajectory_evaluator/dtw.h"

using namespace DTW;

SimpleDTW::SimpleDTW(unsigned int x_size, unsigned int y_size, double (*distance_eval)(std::vector<double> p1, std::vector<double> p2))
{
    DistanceFn = distance_eval;
    dtw_matrix = NULL;
    initialized = SimpleDTW::Initialize(x_size, y_size);
}

SimpleDTW::SimpleDTW(double (*distance_eval)(std::vector<double> p1, std::vector<double> p2))
{
    DistanceFn = distance_eval;
    dtw_matrix = NULL;
    initialized = false;
}

bool SimpleDTW::Initialize(unsigned int x_size, unsigned int y_size)
{
    x_dim = x_size + 1;
    y_dim = y_size + 1;
    // First, we need to deallocate any existing matrix
    if (dtw_matrix != NULL)
    {
        for (unsigned int i = 0; i < x_dim; i++)
        {
            if (dtw_matrix[i] != NULL)
            {
                free(dtw_matrix[i]);
            }
        }
        free(dtw_matrix);
    }
    // Allocate the dtw matrix
    dtw_matrix = (double **) malloc(x_dim * sizeof(double *));
    //Safety check
    if (dtw_matrix == NULL)
    {
        return false;
    }
    //Allocate the empty matrices
    for (unsigned int i = 0; i < x_dim; i++)
    {
        dtw_matrix[i] = (double *) malloc(y_dim * sizeof(double));
        //Safety check
        if (dtw_matrix[i] == NULL)
        {
            return false;
        }
    }
    //Populate matrix with starting values
    dtw_matrix[0][0] = 0.0;
    for (unsigned int i = 1; i < x_dim; i++)
    {
        dtw_matrix[i][0] = INFINITY;
    }
    for (unsigned int i = 1; i < y_dim; i++)
    {
        dtw_matrix[0][i] = INFINITY;
    }
    initialized = true;
    return true;
}

double SimpleDTW::EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2)
{
    // Sanity checks
    if (sequence_1.size() == 0 || sequence_2.size() == 0)
    {
        return INFINITY;
    }
    if (sequence_1[0].size() != sequence_2[0].size())
    {
        throw std::invalid_argument("Sequences for evaluation have different element sizes");
    }
    // Safety checks
    if (!initialized || sequence_1.size() >= x_dim || sequence_2.size() >= y_dim)
    {
        std::cout << "Automatically resizing DTW matrix to fit arguments" << std::endl;
        SimpleDTW::Initialize(sequence_1.size(), sequence_2.size());
    }
    //Compute DTW cost for the two sequences
    for (unsigned int i = 1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j = 1; j <= sequence_2.size(); j++)
        {
            double index_cost = DistanceFn(sequence_1[i - 1], sequence_2[j - 1]);
            double prev_cost;
            if (dtw_matrix[i - 1][j] < dtw_matrix[i - 1][j - 1] && dtw_matrix[i - j][j] < dtw_matrix[i][j - 1])
            {
                prev_cost = dtw_matrix[i - 1][j];
            }
            else if (dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j] && dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j - 1])
            {
                prev_cost = dtw_matrix[i][j - 1];
            }
            else
            {
                prev_cost = dtw_matrix[i - 1][j - 1];
            }
            dtw_matrix[i][j] = index_cost + prev_cost;
        }
    }
    //Return total path cost
    return dtw_matrix[sequence_1.size()][sequence_2.size()];
}

SimpleDTW::~SimpleDTW()
{
    //Clean up the dtw matrix
    if (dtw_matrix != NULL)
    {
        for (unsigned int i = 0; i < x_dim; i++)
        {
            if (dtw_matrix[i] != NULL)
            {
                free(dtw_matrix[i]);
            }
        }
        free(dtw_matrix);
    }
}
