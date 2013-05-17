#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <dirent.h>
#include "math.h"
#include <stdexcept>
#include "trajectory_evaluator/dtw.h"
#include "trajectory_evaluator/xtf.h"
#include "trajectory_evaluator/evaluator.h"

double euclidean_distance(std::vector<double> P1, std::vector<double> P2)
{
    double total = 0.0;
    for (unsigned int i = 0; i < P1.size(); i++)
    {
        total = total + pow((P1[i] - P2[i]), 2);
    }
    return sqrt(total);
}

int omp_test(void)
{
    int th_id, nthreads;
    #pragma omp parallel private(th_id)
    {
        th_id = omp_get_thread_num();
        #pragma omp barrier
        if (th_id == 0)
        {
            nthreads = omp_get_num_threads();
        }
    }
    return nthreads;
}

int compare_performance(int traj_length, int iterations)
{
    struct timespec bstv, betv;
    printf("Building test arrays\n");
    double ** test_seq_1 = (double **)malloc(traj_length * sizeof(double*));
    for (int i = 0; i < traj_length; i++)
    {
        test_seq_1[i] = (double *)malloc(3 * sizeof(double));
        test_seq_1[i][0] = 0.0;
        test_seq_1[i][1] = 0.0;
        test_seq_1[i][2] = 0.0;
    }
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < traj_length; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    double *** test_seq_2 = (double ***)malloc(iterations * sizeof(double**));
    for (int i = 0; i < iterations; i++)
    {
        test_seq_2[i] = (double **)malloc(traj_length * sizeof(double*));
        for (int j = 0; j < traj_length; j++)
        {
            test_seq_2[i][j] = (double *)malloc(3 * sizeof(double));
            test_seq_2[i][j][0] = (double)rand();
            test_seq_2[i][j][1] = (double)rand();
            test_seq_2[i][j][2] = (double)rand();
        }
    }
    std::vector< std::vector< std::vector<double> > > test_vec_2;
    for (int i = 0; i < iterations; i++)
    {
        std::vector< std::vector<double> > traj;
        for (int j = 0; j < traj_length; j++)
        {
            std::vector<double> state2;
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            traj.push_back(state2);
        }
        test_vec_2.push_back(traj);
    }
    DTW::SimpleDTW my_eval = DTW::SimpleDTW(traj_length, traj_length, euclidean_distance);
    printf("Evaluating\n");
    //Run tests
    printf("-----Test single-threaded version-----\n");
    printf("Testing vector variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);
    double scost = 0.0;
    for (int i = 0; i < iterations; i++)
    {
        scost = my_eval.EvaluateCost(test_vec_1, test_vec_2[i]);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
    //---------------------------------------------
    //-----Compute runtimes (single-threaded)--------;
    float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
    bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;
    printf("Final cost: %f\n", scost);
    printf("SINGLE (vector): %f\n", bsecsv);
    return 0;
}

int main()
{
    std::cout << "Launching trajectory evaluator with " << omp_test() << " reported threads from OpenMP" << std::endl;
    std::cout << "Running evaluator tests..." << std::endl;
    EVAL::PoseEvaluator evaler("./");
    XTF::Parser test_parser;
    XTF::Trajectory test_traj = test_parser.ParseTraj("pose_recorded.xtf");
    std::cout << "...loaded a test trajectory..." << std::endl;
    EVAL::EvaluationResult match = evaler.EvaluateAgainstLibrary(test_traj, "desired", "position");
    std::cout << "Evaluation match keys: " << match << std::endl;
    std::cout << "...done evaluator testing" << std::endl;
    std::cout << "Running performance tests..." << std::endl;
    compare_performance(1000, 1000);
    std::cout << "...done performance testing" << std::endl;
    return 0;
}

