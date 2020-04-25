#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse_solution(4);
    rmse_solution << 0, 0, 0, 0;
    // checking the validity of the input by ensuring that the vector size is not zero
    // and also checking that they are both of the same size
    if (estimations.size() != 0 && (estimations.size() == ground_truth.size()))
    {

        for (int i = 0; i < estimations.size(); ++i)
        {
            // calculating the estimate - ground truth
            VectorXd residual = estimations[i] - ground_truth[i];
            //obtaining the sqaure of the residual
            residual = residual.array() * residual.array();
            // Updating our solution
            rmse_solution+= residual;
        }

        // calculating the mean
        rmse_solution = rmse_solution / rmse_solution.size();

        rmse_solution.array().sqrt();
        return rmse_solution;
    }

    // returns an empty solution in the case that it did not meet the initial requirements
    return rmse_solution;

}

// Calculated the Jacobian Matrix given the vector x_state that will contains the position and vector values
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    // Initializing solution matrix
    MatrixXd jacobian_solution (3, 4);

    // locally sotring the individual values from incoming vector
    // The values below pertain to positions of x, y and the vectors of x and y
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Doing simple calculations that will be useful for later implementations.
    float c1 = px * px + py * py;
    float c2 = sqrtf(c1);
    float c3 = (c1 * c2);

    // check division by 0. Will quickly return an empty solution if that is the case
    if(fabs(c1) < 0.0001)
    {
        return jacobian_solution;
    }

    // Computing the Jacobian matrix
    jacobian_solution << (px / c2), (py / c2), 0, 0,
                        -(py / c1), (px / c1), 0, 0,
                        py * (vx * py - vy * px)/c3, px * (vy * px - vx * py)/c3, px / c2, py / c2;

    return jacobian_solution;

}
