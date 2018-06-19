
/** @file FusionEKF.h
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#ifndef _FusionEKF_h_
#define _FusionEKF_h_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF
{
public:

    /** @brief Constructor
    */
    FusionEKF();

    /** @brief Destructor
    */
    virtual ~FusionEKF();

    /** @brief Run the whole flow of the Kalman Filter from here
    *  @param[in] measurement_pack new measurement data
    */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
    * Kalman Filter update and prediction math lives in here.
    */
    KalmanFilter ekf();
    
protected:
    
    bool						_is_initialized;								//!< check whether the tracking toolbox was initialized or not (first measurement)
    long long					_previous_timestamp;							//!< previous timestamp
    Tools						_tools;										//!<  tool object used to compute Jacobian and RMSE
    Eigen::MatrixXd				_R_laser;										//!< laser measurement covariance matrix
    Eigen::MatrixXd				_R_radar;									//!< radar measurement covariance matrix
    Eigen::MatrixXd				_H_laser;										//!< H matrix for laer measurement
    KalmanFilter 				_ekf;										//!< Kalman Filter math object
    double						_noise_ax;									//!< movement noise over x axis
    double						_noise_ay;									//!< movement noise over y axis
    
};

#endif /* _FusionEKF_h_ */
