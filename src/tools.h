
/** @file tools.h
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#ifndef _tools_h_
#define _tools_h_

#include <vector>
#include "Eigen/Dense"

/** @class Tools
 *  @brief usefule tools for Kalman filter calculation
 */
class Tools
{
public:
    
    /** @brief Constructor
     */
    Tools();

    /** @brief Destructor
     */
    virtual ~Tools();

    /** @brief A helper method to calculate RMSE
     *  @param[in] estimations vector with estimated data
     *  @param[in] ground_truth vector with real data
     *  @return result RMSE
     */
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

    /** @brief A helper method to calculate Jacobians
     *  @param[in] x_state current state vector
     *  @return Jacobian matrix
     */
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

    /** @brief h function for radar measurement
     */
    static Eigen::VectorXd Radar_hFunction(const Eigen::VectorXd x);
    
//     static void norm(float &agle);

};

#endif /* _tools_h_ */
