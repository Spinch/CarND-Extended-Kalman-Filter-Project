
/** @file kalman_filter.cpp
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#include <math.h>
#include "tools.h"
#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    _p2h = NULL;
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in)
{
    _x = x_in;
    _P = P_in;
    _F = F_in;
    _I = Eigen::MatrixXd::Identity(_x.size(), _x.size());
    
    return;
}

void KalmanFilter::Predict(double delta_T)
{
    _F(0,2) = delta_T;
    _F(1,3) = delta_T;
    
    _x = _F * _x;
    _P = _F * _P * _F.transpose() + _Q;
}

void KalmanFilter::Update(const Eigen::VectorXd &z)
{
    Eigen::VectorXd y = z - _H*_x;
    Eigen::MatrixXd S = _H * _P * _H.transpose() + _R;
    Eigen::MatrixXd K = _P * _H.transpose() * S.inverse();
    
    //new estimate
    _x = _x + (K * y);
    _P = (_I - K * _H) * _P;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z)
{
    Eigen::VectorXd hx = Eigen::VectorXd(3);
    if (_p2h != NULL)
	hx = (*_p2h)(_x);
    
    Eigen::VectorXd y = z - hx;
    
    // I don't like how this part is coded, but we have to check angle within [-pi,pi],
    // it would be better to write class 'angle', which will automaticaly check it's range,
    // but I won't do it now :)
    if (y(1) > 2*M_PI)
	y(1) -= 2*M_PI;
    else if (y(1) < -2*M_PI)
	y(1) += 2*M_PI;
    
    Eigen::MatrixXd S = _H * _P * _H.transpose() + _R;
    Eigen::MatrixXd K = _P * _H.transpose() * S.inverse();
    
    //new estimate
    _x = _x + (K * y);
    _P = (_I - K * _H) * _P;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q)
{
    _Q = Q;
    return;
}

void KalmanFilter::SetR(Eigen::MatrixXd R)
{
    _R = R;
    return;
}

void KalmanFilter::SetH(Eigen::MatrixXd H)
{
    _H = H;
    return;
}

Eigen::VectorXd &KalmanFilter::x()
{
    return _x;
}

void KalmanFilter::Set_hFunction(Eigen::VectorXd (*p2h)(const Eigen::VectorXd ))
{
    _p2h = p2h;
    return;
}
