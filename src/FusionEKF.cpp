
/** @file FusionEKF.cpp
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "FusionEKF.h"

FusionEKF::FusionEKF()
{
    _is_initialized = false;
    _previous_timestamp = 0;

    // initializing matrices
    _R_laser = Eigen::MatrixXd(2, 2);
    _R_radar = Eigen::MatrixXd(3, 3);
    _H_laser = Eigen::MatrixXd(2, 4);

    //measurement covariance matrix - laser
    _R_laser << 0.0225, 0,
		      0, 0.0225;

    //measurement covariance matrix - radar
    _R_radar << 0.09, 0, 0,
		       0, 0.0009, 0,
		       0, 0, 0.09;

    _H_laser << 1, 0, 0, 0,
		        0, 1, 0, 0;
   
    _noise_ax = 9;
    _noise_ay = 9;
}

FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    // IInitialization
    if (!_is_initialized)
    {	
	Eigen::VectorXd x = Eigen::VectorXd(4);;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
	    x(0) = measurement_pack.raw_measurements_(0);
	    x(1) = measurement_pack.raw_measurements_(1);
	    x(2) = 0;
	    x(3) = 0;
	}
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
	{
	    x(0) = measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
	    x(1) = measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
	    x(2) = 0;
	    x(3) = 0;
	}
	
	_previous_timestamp = measurement_pack.timestamp_;
	
	Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
	P << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1000, 0,
		  0, 0, 0, 1000;
	
	Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
	F << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	
        _ekf.Init(x, P, F);
	
	Eigen::VectorXd (*p2h)(const Eigen::VectorXd ) = &(Tools::Radar_hFunction);
	_ekf.Set_hFunction(p2h);
	
	// done initializing, no need to predict or update
	_is_initialized = true;
	return;
   }

   // Prediction
    float dt = (measurement_pack.timestamp_ - _previous_timestamp) / 1000000.0;	//dt - expressed in seconds
    _previous_timestamp = measurement_pack.timestamp_;
    float dt2 = dt*dt;
    float dt3_2 = dt2*dt/2;
    float dt4_4 = dt3_2*dt/2;
    Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
    Q << dt4_4*_noise_ax,   	0,              		dt3_2*_noise_ax, 		0,
		0,               		 dt4_4*_noise_ay, 	0,              			dt3_2*_noise_ay,
		dt3_2*_noise_ax,   	0,              		dt2*_noise_ax,   		0,
		0,                		dt3_2*_noise_ay, 	0,              			dt2*_noise_ay;
    
    _ekf.SetQ(Q);
    _ekf.Predict(dt);

   // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
	// Radar updates
	  _ekf.SetR(_R_radar);
	  _ekf.SetH(Tools::CalculateJacobian(_ekf.x()));
	  _ekf.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
	// Laser updates
	_ekf.SetR(_R_laser);
	_ekf.SetH(_H_laser);
	_ekf.Update(measurement_pack.raw_measurements_);
    }

//   // print the output
//   cout << "x_ = " << ekf_.x_ << endl;
//   cout << "P_ = " << ekf_.P_ << endl;
}

KalmanFilter FusionEKF::ekf()
{
    return _ekf;
}
