
/** @file kalman_filter.h
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#ifndef _kalman_filter_h_
#define _kalman_filter_h_

#include "Eigen/Dense"

/** @class KalmanFilter
 *  @breif Kalman filter calculations
 */
class KalmanFilter
{
public:

    /** @brief Constructor
     */
    KalmanFilter();

    /** @brief Destructor
     */
    virtual ~KalmanFilter();

    /** @brief Initializes Kalman filter
     * @param[in] x_in Initial state
     * @param[in] P_in Initial state covariance
     * @param[in] F_in Transition matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in);

    /** @brief Predicts the state and the state covariance using the process model
     *  @param delta_T Time between k and k+1 in s
     */
    void Predict(double delta_T);

    /** @brief Updates the state by using standard Kalman Filter equations
    *  @param[in] z The measurement at k+1
    */
    void Update(const Eigen::VectorXd &z);

    /** @brief Updates the state by using Extended Kalman Filter equations
     * @param[in] z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);
    
    /** @breif Set matrix Q
     *  @param[in] Q new movement noise matrix
     */
    void SetQ(Eigen::MatrixXd Q);
    
    /** @breif Set matrix R
     *  @param[in] R new measurement noise matrix
     */
    void SetR(Eigen::MatrixXd R);
    
    /** @breif Set matrix H
     *  @param[in] H new matrix H
     */
    void SetH(Eigen::MatrixXd H);
    
    /** @brief Get current state vector
     *  @return current state vector
     */
    Eigen::VectorXd &x();
    
    /** @breif Method to set h function to use by EKF
     *  @param[in] p2h pointer to h function
     */
    void Set_hFunction(Eigen::VectorXd (*p2h)(const Eigen::VectorXd ));
    
protected:
    
    Eigen::VectorXd							_x;								//!< state vector
    Eigen::MatrixXd							_P;								//!< state covariance matrix
    Eigen::MatrixXd							_F;								//!< state transition matrix
    Eigen::MatrixXd							_Q;								//!< process covariance matrix
    Eigen::MatrixXd							_H;								//!< measurement matrix
    Eigen::MatrixXd							_R;								//!< measurement covariance matrix
    Eigen::MatrixXd							_I;								//!< identity matrix
    Eigen::VectorXd 							(*_p2h)(const Eigen::VectorXd );		//!< pointer to function h for EKF
    
};

#endif /* _kalman_filter_h_ */
