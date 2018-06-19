
/** @file tools.cpp
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#include <iostream>
#include "tools.h"

Tools::Tools()
{
}

Tools::~Tools()
{
}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth)
{
    Eigen::VectorXd rmse(4);
    rmse << 0,0,0,0;

    if ( (estimations.size() == 0) || (estimations.size() != ground_truth.size()) )
	return rmse;

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i)
    {
	Eigen::VectorXd d = estimations[i] - ground_truth[i];
	d = d.array()*d.array();
	rmse += d;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state)
{
    Eigen::MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //check division by zero
    if ((fabs(px)<0.001) && (fabs(py)<0.001))
    {
	std::cout << "Error!" << std::endl;
	return Hj;
    }
    
    //compute the Jacobian matrix
    float norm2 = px*px + py*py;
    float norm = sqrt(norm2);
    
    Hj << px/norm,      py/norm,      0, 0,
	  -py/norm2,    px/norm2,     0, 0,
	  py*(vx*py-vy*px)/(norm*norm2), px*(vy*px-vx*py)/(norm*norm2),
	    px/norm, py/norm;

    return Hj;
}

Eigen::VectorXd Tools::Radar_hFunction(const Eigen::VectorXd x)
{
    Eigen::VectorXd hx = Eigen::VectorXd(3);
    hx(0) = sqrt(x(0)*x(0) + x(1)*x(1));
    if (hx(0) < 0.001)
    {
	hx << 0,0,0;
	return hx;
    }
        hx(1) = atan2(x(1), x(0));
	hx(2) = (x(0)*x(2) + x(1)*x(3)) / hx(0);
	return hx;
}

// void Tools::norm(float& agle)
// {
//     if (agle > 2*M_PI)
// 	agle -= 2*M_PI;
//     else if (agle < -2*M_PI)
// 	agle += 2*M_PI;
//     return;
// }
