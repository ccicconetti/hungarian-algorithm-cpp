///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm
// implementation by Markus Buehren. The original implementation is a few
// mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//
// minor changes by C. Cicconetti, 2022
//

#pragma once

#include <functional>
#include <iostream>
#include <vector>

namespace hungarian {

/**
 * @brief Solve the assignment problem with the Hungarian algorithm.
 */
class HungarianAlgorithm final
{
 public:
  using DistMatrix = std::vector<std::vector<double>>;

  /**
   * @brief Solve the assignment problem with the Hungarian algorithm.
   *
   * @param DistMatrix The input cost matrix, i.e., the cost to execute a given
   * task on each worker.
   * @param Assignment The assignment found. A value of -1 is used for tasks not
   * assigned to workers. The vector is resized to match the number of tasks.
   * @return double The assignment cost.
   */
  static double Solve(const DistMatrix& DistMatrix,
                      std::vector<int>& Assignment);

  /**
   * @brief Assign each task to the worker with the smallest cost.
   *
   * @param DistMatrix The input cost matrix, i.e., the cost to execute a given
   * task on each worker.
   * @param Assignment The assignment found. A value of -1 is used for tasks not
   * assigned to workers. The vector is resized to match the number of tasks.
   * @param Rnd A function that returns a random value.
   * @return double The assignment cost.
   */
  static double SolveRandom(const DistMatrix&              DistMatrix,
                            std::vector<int>&              Assignment,
                            const std::function<double()>& aRnd);

  /**
   * @brief Assigning tasks to workers at random.
   *
   * @param DistMatrix The input cost matrix, i.e., the cost to execute a given
   * task on each worker.
   * @param Assignment The assignment found. A value of -1 is used for tasks not
   * assigned to workers. The vector is resized to match the number of tasks.
   * @return double The assignment cost.
   */
  static double SolveGreedy(const DistMatrix& DistMatrix,
                            std::vector<int>& Assignment);

 private:
  /**
   * @brief Check if the input cost matrix is valid and return the size.
   *
   * @param DistMatrix The input cost matrix.
   * @return the number of rows and number of columns of the input matrix.
   *
   * @throw std::runtime_error if the input is not valid
   */
  static std::pair<unsigned int, unsigned int>
  checkInput(const DistMatrix& DistMatrix);

  static void assignmentoptimal(int*    assignment,
                                double* cost,
                                double* distMatrix,
                                int     nOfRows,
                                int     nOfColumns);
  static void buildassignmentvector(int*  assignment,
                                    bool* starMatrix,
                                    int   nOfRows,
                                    int   nOfColumns);
  static void computeassignmentcost(int*    assignment,
                                    double* cost,
                                    double* distMatrix,
                                    int     nOfRows);
  static void step2a(int*    assignment,
                     double* distMatrix,
                     bool*   starMatrix,
                     bool*   newStarMatrix,
                     bool*   primeMatrix,
                     bool*   coveredColumns,
                     bool*   coveredRows,
                     int     nOfRows,
                     int     nOfColumns,
                     int     minDim);
  static void step2b(int*    assignment,
                     double* distMatrix,
                     bool*   starMatrix,
                     bool*   newStarMatrix,
                     bool*   primeMatrix,
                     bool*   coveredColumns,
                     bool*   coveredRows,
                     int     nOfRows,
                     int     nOfColumns,
                     int     minDim);
  static void step3(int*    assignment,
                    double* distMatrix,
                    bool*   starMatrix,
                    bool*   newStarMatrix,
                    bool*   primeMatrix,
                    bool*   coveredColumns,
                    bool*   coveredRows,
                    int     nOfRows,
                    int     nOfColumns,
                    int     minDim);
  static void step4(int*    assignment,
                    double* distMatrix,
                    bool*   starMatrix,
                    bool*   newStarMatrix,
                    bool*   primeMatrix,
                    bool*   coveredColumns,
                    bool*   coveredRows,
                    int     nOfRows,
                    int     nOfColumns,
                    int     minDim,
                    int     row,
                    int     col);
  static void step5(int*    assignment,
                    double* distMatrix,
                    bool*   starMatrix,
                    bool*   newStarMatrix,
                    bool*   primeMatrix,
                    bool*   coveredColumns,
                    bool*   coveredRows,
                    int     nOfRows,
                    int     nOfColumns,
                    int     minDim);
};

} // namespace hungarian
